#include "dvc_motor_protect.h"
#include <math.h>
#include <stdint.h>

static uint16_t Motor_Protect_Sat_Add_U16(uint16_t value, uint16_t delta)
{
    uint32_t sum = (uint32_t)value + (uint32_t)delta;

    if (sum > 65535u)
    {
        return 65535u;
    }

    return (uint16_t)sum;
}

static float Motor_Protect_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static uint16_t Motor_Protect_Get_DtMs(const Motor_Protect_Input_TypeDef *input)
{
    if (input == NULL || input->dt_ms == 0u)
    {
        return 1u;
    }

    return input->dt_ms;
}

static void Motor_Protect_Enter_State(Motor_Protect_Handle_TypeDef *handle,
                                      Motor_Protect_State_TypeDef next_state)
{
    if (handle == NULL)
    {
        return;
    }

    handle->state = next_state;
    handle->state_ms = 0u;
    handle->stall_ms = 0u;

    if (next_state == MOTOR_PROTECT_STATE_BACKOFF)
    {
        handle->backoff_request = 1u;
    }
    else
    {
        handle->backoff_request = 0u;
    }
}

static uint8_t Motor_Protect_Check_Stall(Motor_Protect_Handle_TypeDef *handle,
                                         const Motor_Protect_Config_TypeDef *config,
                                         const Motor_Protect_Input_TypeDef *input)
{
    uint8_t vote = 0u;
    uint16_t dt_ms;

    if (handle == NULL || config == NULL || input == NULL)
    {
        return 0u;
    }

    if (handle->state_ms < config->blank_ms)
    {
        handle->stall_ms = 0u;
        return 0u;
    }

    dt_ms = Motor_Protect_Get_DtMs(input);

    if (fabsf(input->cmd) > config->cmd_th)
    {
        vote++;
    }

    if (fabsf((float)input->torque_raw) > config->torque_th)
    {
        vote++;
    }

    if (fabsf(input->speed_rad_s) < config->speed_th_rad_s)
    {
        vote++;
    }

    if (fabsf(input->pos_err_deg) > config->err_th_deg)
    {
        vote++;
    }

    if (vote >= 3u)
    {
        handle->stall_ms = Motor_Protect_Sat_Add_U16(handle->stall_ms, dt_ms);
    }
    else
    {
        handle->stall_ms = 0u;
    }

    if (handle->stall_ms >= config->confirm_ms)
    {
        return 1u;
    }

    return 0u;
}

void Motor_Protect_Init(Motor_Protect_Handle_TypeDef *handle)
{
    Motor_Protect_Reset(handle);
}

void Motor_Protect_Reset(Motor_Protect_Handle_TypeDef *handle)
{
    if (handle == NULL)
    {
        return;
    }

    handle->state = MOTOR_PROTECT_STATE_NORMAL;
    handle->state_ms = 0u;
    handle->stall_ms = 0u;
    handle->retry_count = 0u;
    handle->backoff_request = 0u;
    handle->last_cmd = 0.0f;
}

void Motor_Protect_Blank(Motor_Protect_Handle_TypeDef *handle)
{
    if (handle == NULL)
    {
        return;
    }

    handle->state_ms = 0u;
    handle->stall_ms = 0u;
}

void Motor_Protect_Clear_Fault(Motor_Protect_Handle_TypeDef *handle)
{
    Motor_Protect_Reset(handle);
}

void Motor_Protect_Update(Motor_Protect_Handle_TypeDef *handle,
                          const Motor_Protect_Config_TypeDef *config,
                          const Motor_Protect_Input_TypeDef *input)
{
    uint16_t dt_ms;

    if (handle == NULL || config == NULL || input == NULL)
    {
        return;
    }

    handle->last_cmd = input->cmd;

    if (config->enable == 0u)
    {
        Motor_Protect_Reset(handle);
        return;
    }

    dt_ms = Motor_Protect_Get_DtMs(input);

    switch (handle->state)
    {
        case MOTOR_PROTECT_STATE_NORMAL:
            handle->state_ms = Motor_Protect_Sat_Add_U16(handle->state_ms, dt_ms);

            if ((config->retry_reset_ms > 0u) &&
                (handle->state_ms >= config->retry_reset_ms))
            {
                handle->retry_count = 0u;
            }

            if (Motor_Protect_Check_Stall(handle, config, input) != 0u)
            {
                if ((config->allow_backoff != 0u) &&
                    (config->has_mech_limit != 0u) &&
                    (input->near_limit != 0u) &&
                    (input->pushing_outward != 0u))
                {
                    if (handle->retry_count < 255u)
                    {
                        handle->retry_count++;
                    }

                    Motor_Protect_Enter_State(handle, MOTOR_PROTECT_STATE_BACKOFF);
                }
                else
                {
                    Motor_Protect_Enter_State(handle, MOTOR_PROTECT_STATE_FAULT);
                }
            }
            break;

        case MOTOR_PROTECT_STATE_BACKOFF:
            handle->state_ms = Motor_Protect_Sat_Add_U16(handle->state_ms, dt_ms);

            if (handle->state_ms >= config->backoff_ms)
            {
                if (handle->retry_count > config->retry_limit)
                {
                    Motor_Protect_Enter_State(handle, MOTOR_PROTECT_STATE_FAULT);
                }
                else
                {
                    Motor_Protect_Enter_State(handle, MOTOR_PROTECT_STATE_COOLDOWN);
                }
            }
            break;

        case MOTOR_PROTECT_STATE_COOLDOWN:
            handle->state_ms = Motor_Protect_Sat_Add_U16(handle->state_ms, dt_ms);

            if (handle->state_ms >= config->cooldown_ms)
            {
                Motor_Protect_Enter_State(handle, MOTOR_PROTECT_STATE_NORMAL);
            }
            break;

        case MOTOR_PROTECT_STATE_FAULT:
        default:
            break;
    }
}

float Motor_Protect_Apply_Output(const Motor_Protect_Handle_TypeDef *handle,
                                 const Motor_Protect_Config_TypeDef *config,
                                 float raw_cmd)
{
    float limit;

    if (handle == NULL || config == NULL || config->enable == 0u)
    {
        return raw_cmd;
    }

    switch (handle->state)
    {
        case MOTOR_PROTECT_STATE_BACKOFF:
            limit = fabsf(config->backoff_cmd_limit);
            if (limit <= 0.0f)
            {
                return 0.0f;
            }
            return Motor_Protect_Clamp(raw_cmd, -limit, limit);

        case MOTOR_PROTECT_STATE_COOLDOWN:
        case MOTOR_PROTECT_STATE_FAULT:
            return 0.0f;

        case MOTOR_PROTECT_STATE_NORMAL:
        default:
            return raw_cmd;
    }
}

uint8_t Motor_Protect_Consume_Backoff_Request(Motor_Protect_Handle_TypeDef *handle)
{
    uint8_t request;

    if (handle == NULL)
    {
        return 0u;
    }

    request = handle->backoff_request;
    handle->backoff_request = 0u;
    return request;
}

uint8_t Motor_Protect_Is_Fault(const Motor_Protect_Handle_TypeDef *handle)
{
    if ((handle != NULL) && (handle->state == MOTOR_PROTECT_STATE_FAULT))
    {
        return 1u;
    }

    return 0u;
}
