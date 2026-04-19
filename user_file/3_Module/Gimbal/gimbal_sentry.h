#ifndef __GIMBAL_SENTRY_H__
#define __GIMBAL_SENTRY_H__

#include <stdint.h>

typedef enum
{
    // 无目标时的搜索状态：按正弦轨迹生成扫描目标
    GIMBAL_SENTRY_STATE_SCAN = 0,
    // 视觉目标新鲜时的跟踪状态：直接跟随视觉给出的目标角
    GIMBAL_SENTRY_STATE_TRACK_ARMOR,
    // 目标丢失后的过渡状态：先回到扫描轨迹，再继续扫描
    GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN,
} Gimbal_Sentry_State_TypeDef;

typedef struct
{
    // 控制周期，单位秒；当前由 Gimbal.c 传入 0.001s
    float dt_s;

    // 云台物理/安全角度边界，用于最终夹紧目标
    float pitch_min_deg;
    float pitch_max_deg;
    float yaw_min_deg;
    float yaw_max_deg;

    // 扫描正弦波参数：默认以 0 度为中心，通过幅值和频率调节扫描范围与快慢
    float scan_pitch_amplitude_deg;
    float scan_yaw_amplitude_deg;
    float scan_pitch_frequency_hz;
    float scan_yaw_frequency_hz;

    // 丢目标回扫参数
    float lost_return_speed_deg_s;
    float lost_return_near_deg;
} Gimbal_Sentry_Config_TypeDef;

/**
 * @brief 哨兵状态机的视觉输入。
 * @note  上层已经把 Target_Valid、最后一次有效目标保持和通信在线判断
 *        统一折叠为 vision_target_available；状态机只消费整理后的输入。
 */
typedef struct
{
    // 由 Gimbal.c 根据有效目标缓存和超时时间判断，模块本身不依赖 USB/协议细节
    uint8_t vision_target_available;
    // 当前可跟踪的视觉目标角；当前协议默认它们是“绝对目标角”
    float vision_pitch_deg;
    float vision_yaw_deg;
} Gimbal_Sentry_Input_TypeDef;

typedef struct
{
    // 模块最终输出给 Gimbal.c 的双轴目标角
    float pitch_target_deg;
    float yaw_target_deg;
    // 对外暴露当前状态，便于调试或后续上报
    Gimbal_Sentry_State_TypeDef state;
} Gimbal_Sentry_Output_TypeDef;

typedef struct
{
    // 当前状态机状态
    Gimbal_Sentry_State_TypeDef state;
    // 扫描正弦波相位，单位 rad
    float scan_yaw_phase_rad;
    float scan_pitch_phase_rad;
    // 扫描轨迹内部目标；即使正在跟踪目标，扫描相位也继续推进
    float scan_yaw_target_deg;
    float scan_pitch_target_deg;
    // 丢目标瞬间冻结的扫描轨迹点，用于平滑回扫
    float lost_return_yaw_target_deg;
    float lost_return_pitch_target_deg;
    // 当前输出目标缓存
    float yaw_target_deg;
    float pitch_target_deg;
} Gimbal_Sentry_Handle_TypeDef;

// 初始化/复位状态机，默认回到 SCAN 且目标归零
void Gimbal_Sentry_Init(Gimbal_Sentry_Handle_TypeDef *handle);
void Gimbal_Sentry_Reset(Gimbal_Sentry_Handle_TypeDef *handle);
// 每个控制周期调用一次，根据视觉输入生成本拍 pitch/yaw 目标角
void Gimbal_Sentry_Update(Gimbal_Sentry_Handle_TypeDef *handle,
                          const Gimbal_Sentry_Config_TypeDef *config,
                          const Gimbal_Sentry_Input_TypeDef *input,
                          Gimbal_Sentry_Output_TypeDef *output);
// 获取当前状态，方便调试或后续通过 VOFA/USB 上报
Gimbal_Sentry_State_TypeDef Gimbal_Sentry_Get_State(const Gimbal_Sentry_Handle_TypeDef *handle);

#endif /* __GIMBAL_SENTRY_H__ */
