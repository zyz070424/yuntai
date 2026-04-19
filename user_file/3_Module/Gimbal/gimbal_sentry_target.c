#include "gimbal_sentry_target.h"
#include "dvc_manifold.h"
#include "drv_usb.h"

extern Manifold_UART_Rx_Data Rx_Data;

static Gimbal_Sentry_Target_Config_TypeDef g_gimbal_sentry_target_config;
static uint8_t g_gimbal_sentry_target_initialized = 0u;
static Gimbal_Sentry_Handle_TypeDef g_gimbal_sentry;
static uint32_t g_gimbal_visual_last_rx_frame_seq = 0u;
static TickType_t g_gimbal_visual_last_valid_tick = 0u;
static float g_gimbal_visual_last_valid_pitch_deg = 0.0f;
static float g_gimbal_visual_last_valid_yaw_deg = 0.0f;
static float g_gimbal_visual_filtered_pitch_deg = 0.0f;
static float g_gimbal_visual_filtered_yaw_deg = 0.0f;
static uint8_t g_gimbal_visual_has_valid_target = 0u;
static uint8_t g_gimbal_visual_filter_tracking = 0u;
static float g_gimbal_pitch_target = 0.0f;
static float g_gimbal_yaw_target = 0.0f;

static float Gimbal_Sentry_Target_Clamp(float value, float min_value, float max_value)
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

static float Gimbal_Sentry_Target_First_Order_Low_Pass(float current_value,
                                                       float target_value,
                                                       float tau_s,
                                                       float dt_s)
{
    float alpha;

    if (tau_s <= 0.0f)
    {
        return target_value;
    }

    alpha = dt_s / (tau_s + dt_s);
    alpha = Gimbal_Sentry_Target_Clamp(alpha, 0.0f, 1.0f);
    return current_value + alpha * (target_value - current_value);
}

static void Gimbal_Sentry_Target_Update_Cache(TickType_t now_tick)
{
    uint32_t rx_frame_seq;

    rx_frame_seq = Manifold_USB_Rx_Frame_Seq;
    if (rx_frame_seq != g_gimbal_visual_last_rx_frame_seq)
    {
        g_gimbal_visual_last_rx_frame_seq = rx_frame_seq;
        if (Rx_Data.Target_Valid != 0u)
        {
            g_gimbal_visual_last_valid_pitch_deg = Rx_Data.Taget_Pitch;
            g_gimbal_visual_last_valid_yaw_deg = Rx_Data.Taget_Yaw;
            g_gimbal_visual_last_valid_tick = now_tick;
            g_gimbal_visual_has_valid_target = 1u;
        }
    }
}

static uint8_t Gimbal_Sentry_Target_IsAvailable(TickType_t now_tick)
{
    Gimbal_Sentry_Target_Update_Cache(now_tick);

    if ((g_gimbal_visual_has_valid_target == 0u) ||
        (USB_Alive_IsRxOnline() == 0u))
    {
        return 0u;
    }

    if ((now_tick - g_gimbal_visual_last_valid_tick) >
        pdMS_TO_TICKS(g_gimbal_sentry_target_config.vision_track_timeout_ms))
    {
        return 0u;
    }

    return 1u;
}

static void Gimbal_Sentry_Target_Update_Filter(uint8_t target_available)
{
    if (target_available == 0u)
    {
        g_gimbal_visual_filter_tracking = 0u;
        return;
    }

    if (g_gimbal_visual_filter_tracking == 0u)
    {
        g_gimbal_visual_filtered_pitch_deg = g_gimbal_visual_last_valid_pitch_deg;
        g_gimbal_visual_filtered_yaw_deg = g_gimbal_visual_last_valid_yaw_deg;
        g_gimbal_visual_filter_tracking = 1u;
        return;
    }

    g_gimbal_visual_filtered_pitch_deg = Gimbal_Sentry_Target_First_Order_Low_Pass(
        g_gimbal_visual_filtered_pitch_deg,
        g_gimbal_visual_last_valid_pitch_deg,
        g_gimbal_sentry_target_config.vision_target_filter_tau_s,
        g_gimbal_sentry_target_config.control_dt_s);
    g_gimbal_visual_filtered_yaw_deg = Gimbal_Sentry_Target_First_Order_Low_Pass(
        g_gimbal_visual_filtered_yaw_deg,
        g_gimbal_visual_last_valid_yaw_deg,
        g_gimbal_sentry_target_config.vision_target_filter_tau_s,
        g_gimbal_sentry_target_config.control_dt_s);
}

void Gimbal_Sentry_Target_Init(const Gimbal_Sentry_Target_Config_TypeDef *config)
{
    if (config == NULL)
    {
        return;
    }

    g_gimbal_sentry_target_config = *config;
    g_gimbal_sentry_target_initialized = 1u;
    Gimbal_Sentry_Target_Reset_Mode();
}

void Gimbal_Sentry_Target_Reset_Mode(void)
{
    if (g_gimbal_sentry_target_initialized == 0u)
    {
        return;
    }

    Gimbal_Sentry_Reset(&g_gimbal_sentry);
    Gimbal_Sentry_Target_Reset_Vision();
    Gimbal_Sentry_Target_Clear_Output();
}

void Gimbal_Sentry_Target_Reset_Vision(void)
{
    g_gimbal_visual_last_rx_frame_seq = Manifold_USB_Rx_Frame_Seq;
    g_gimbal_visual_last_valid_tick = 0u;
    g_gimbal_visual_last_valid_pitch_deg = 0.0f;
    g_gimbal_visual_last_valid_yaw_deg = 0.0f;
    g_gimbal_visual_filtered_pitch_deg = 0.0f;
    g_gimbal_visual_filtered_yaw_deg = 0.0f;
    g_gimbal_visual_has_valid_target = 0u;
    g_gimbal_visual_filter_tracking = 0u;
}

void Gimbal_Sentry_Target_Clear_Output(void)
{
    g_gimbal_pitch_target = 0.0f;
    g_gimbal_yaw_target = 0.0f;
}

void Gimbal_Sentry_Target_Update(TickType_t now_tick)
{
    Gimbal_Sentry_Input_TypeDef sentry_input;
    Gimbal_Sentry_Output_TypeDef sentry_output;

    if (g_gimbal_sentry_target_initialized == 0u)
    {
        return;
    }

    sentry_input.vision_target_available = Gimbal_Sentry_Target_IsAvailable(now_tick);
    Gimbal_Sentry_Target_Update_Filter(sentry_input.vision_target_available);
    sentry_input.vision_pitch_deg = g_gimbal_visual_filtered_pitch_deg;
    sentry_input.vision_yaw_deg = g_gimbal_visual_filtered_yaw_deg;

    Gimbal_Sentry_Update(&g_gimbal_sentry,
                         &g_gimbal_sentry_target_config.sentry_config,
                         &sentry_input,
                         &sentry_output);
    g_gimbal_pitch_target = -sentry_output.pitch_target_deg;
    g_gimbal_yaw_target = sentry_output.yaw_target_deg;
}

float Gimbal_Sentry_Target_Get_Pitch(void)
{
    return g_gimbal_pitch_target;
}

float Gimbal_Sentry_Target_Get_Yaw(void)
{
    return g_gimbal_yaw_target;
}

Gimbal_Sentry_State_TypeDef Gimbal_Sentry_Target_Get_State(void)
{
    if (g_gimbal_sentry_target_initialized == 0u)
    {
        return GIMBAL_SENTRY_STATE_SCAN;
    }

    return Gimbal_Sentry_Get_State(&g_gimbal_sentry);
}
