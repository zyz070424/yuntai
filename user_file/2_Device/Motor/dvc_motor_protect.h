#ifndef __DVC_MOTOR_PROTECT_H__
#define __DVC_MOTOR_PROTECT_H__

#include <stdint.h>

/*
 * motor protect module only owns "stall detect / backoff / cooldown / fault".
 * It does not own gimbal scan targets, endpoint planning, or PID reset policy.
 */

typedef enum
{
    MOTOR_PROTECT_STATE_NORMAL = 0,
    MOTOR_PROTECT_STATE_BACKOFF,
    MOTOR_PROTECT_STATE_COOLDOWN,
    MOTOR_PROTECT_STATE_FAULT,
} Motor_Protect_State_TypeDef;

typedef struct
{
    uint8_t enable;
    uint8_t allow_backoff;
    uint8_t has_mech_limit;

    float cmd_th;
    float torque_th;
    float speed_th_rad_s;
    float err_th_deg;

    uint16_t blank_ms;
    uint16_t confirm_ms;
    uint16_t backoff_ms;
    uint16_t cooldown_ms;
    uint16_t retry_reset_ms;

    float backoff_cmd_limit;
    uint8_t retry_limit;
} Motor_Protect_Config_TypeDef;

typedef struct
{
    Motor_Protect_State_TypeDef state;
    uint16_t state_ms;
    uint16_t stall_ms;
    uint8_t retry_count;
    uint8_t backoff_request;
    float last_cmd;
} Motor_Protect_Handle_TypeDef;

typedef struct
{
    uint16_t dt_ms;
    float pos_err_deg;
    float speed_rad_s;
    float cmd;
    int16_t torque_raw;
    uint8_t near_limit;
    uint8_t pushing_outward;
} Motor_Protect_Input_TypeDef;

void Motor_Protect_Init(Motor_Protect_Handle_TypeDef *handle);
void Motor_Protect_Reset(Motor_Protect_Handle_TypeDef *handle);
void Motor_Protect_Blank(Motor_Protect_Handle_TypeDef *handle);
void Motor_Protect_Clear_Fault(Motor_Protect_Handle_TypeDef *handle);
void Motor_Protect_Update(Motor_Protect_Handle_TypeDef *handle,
                          const Motor_Protect_Config_TypeDef *config,
                          const Motor_Protect_Input_TypeDef *input);
float Motor_Protect_Apply_Output(const Motor_Protect_Handle_TypeDef *handle,
                                 const Motor_Protect_Config_TypeDef *config,
                                 float raw_cmd);
uint8_t Motor_Protect_Consume_Backoff_Request(Motor_Protect_Handle_TypeDef *handle);
uint8_t Motor_Protect_Is_Fault(const Motor_Protect_Handle_TypeDef *handle);

#endif /* __DVC_MOTOR_PROTECT_H__ */
