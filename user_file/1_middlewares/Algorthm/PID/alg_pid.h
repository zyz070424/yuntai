#ifndef __ALG_PID_H__
#define __ALG_PID_H__

#include "main.h"
#include <stdbool.h>
#include <math.h>
#include <sys/types.h>
typedef struct {
    // ===== 1. PID参数 =====
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float FeedForward;  // 前馈系数
    
    float P_out;         // 比例输出
    float I_out;         // 积分输出
    float D_out;         // 微分输出
    float FeedForward_out; // 前馈输出
    
    // ===== 2. 运行状态 =====
    float target;     // 目标值
    float prev_target;  // 上一次目标值（用于微分先行 和 前馈）
    float output;       // 控制输出
    float Input;        // 当前输入值
    
    // ===== 3. 误差相关 =====
    float error;        // 当前误差
    float prev_error;   // 上一次误差（用于微分）
    float integral;     // 积分累加和
    
    
    // ===== 4. 输出限幅 =====
    float out_min;      // 输出最小值
    float out_max;      // 输出最大值
    
    // ===== 5. 积分限幅（抗积分饱和）=====
    float integral_min; // 积分项最小值
    float integral_max; // 积分项最大值
    
    // ===== 6. 目标限幅 =====
    bool target_limit_enable;  // 目标限幅使能
    float target_limit_min;    // 目标限幅最小值
    float target_limit_max;    // 目标限幅最大值

    // ===== 7. 变速积分 =====
    bool integral_separation_enable; // 积分分离使能
    float integral_separation_threshold_A; // 积分分离阈值
    float integral_separation_threshold_B; // 积分分离阈值

    // ===== 8. 微分先行 =====
    bool differential_enable; // 微分先行使能

    // ===== 9. 死区 =====
    bool deadband_enable; // 死区使能
    float deadband; // 死区临界值值

    // ===== 10. 输出整形（可选）=====
    bool output_filter_enable; // 输出低通使能
    float output_filter_tau_s; // 输出低通时间常数（秒）
    bool output_slew_enable;   // 输出斜率限制使能
    float output_slew_rate;    // 输出最大变化率（单位/秒）
    bool output_shaper_inited; // 输出整形状态是否已初始化
    float output_shaper_state; // 输出整形内部状态

    // ===== 11. 时间间隔 =====
    float dt; // 时间间隔（单位：秒）
} PID_TypeDef;
void PID_Init(PID_TypeDef *pid);

void PID_Differential_Enable(PID_TypeDef *pid,bool enable);

void PID_Deadband_Enable(PID_TypeDef *pid,bool enable,float deadband);

void PID_Output_Filter_Enable(PID_TypeDef *pid, bool enable, float tau_s);

void PID_Output_Slew_Enable(PID_TypeDef *pid, bool enable, float slew_rate);

void PID_Output_Shaper_Reset(PID_TypeDef *pid, float init_output);

void PID_Integral_Separation_Enable(PID_TypeDef *pid,bool enable,float threshold_A,float threshold_B);

void PID_Target_Limit_Enable(PID_TypeDef *pid,bool enable,float min,float max);

void PID_Set_Parameters(PID_TypeDef *pid,float P,float I,float D,float FeedForward,float integral_min,float integral_max,float out_min,float out_max);

float PID_Calculate(PID_TypeDef *pid,float Input,float Target,float dt);

#endif /* __ALG_PID_H__ */
