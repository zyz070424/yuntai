#ifndef __GIMBAL_SENTRY_TARGET_H__
#define __GIMBAL_SENTRY_TARGET_H__

#include "FreeRTOS.h"
#include "gimbal_sentry.h"
#include <stdint.h>

typedef struct
{
    float control_dt_s;
    uint32_t vision_track_timeout_ms;
    float vision_target_filter_tau_s;
    Gimbal_Sentry_Config_TypeDef sentry_config;
} Gimbal_Sentry_Target_Config_TypeDef;

void Gimbal_Sentry_Target_Init(const Gimbal_Sentry_Target_Config_TypeDef *config);
void Gimbal_Sentry_Target_Reset_Mode(void);
void Gimbal_Sentry_Target_Reset_Vision(void);
void Gimbal_Sentry_Target_Clear_Output(void);
void Gimbal_Sentry_Target_Update(TickType_t now_tick);
float Gimbal_Sentry_Target_Get_Pitch(void);
float Gimbal_Sentry_Target_Get_Yaw(void);
Gimbal_Sentry_State_TypeDef Gimbal_Sentry_Target_Get_State(void);

#endif /* __GIMBAL_SENTRY_TARGET_H__ */
