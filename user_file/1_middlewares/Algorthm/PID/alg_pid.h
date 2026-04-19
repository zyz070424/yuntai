#ifndef __ALG_PID_H__
#define __ALG_PID_H__

#include "main.h"
#include <stdbool.h>
#include <math.h>
#include <sys/types.h>

/**
 * @brief PID 输出整形调度档位。
 * @note  调度器根据误差大小在 FAST / MEDIUM / FINE 三档之间切换，
 *        用于在大误差时保持响应速度、在近目标区降低抖动。
 */
typedef enum
{
    PID_OUTPUT_SCHEDULE_MODE_FAST = 0,
    PID_OUTPUT_SCHEDULE_MODE_MEDIUM,
    PID_OUTPUT_SCHEDULE_MODE_FINE
} PID_Output_Schedule_Mode_TypeDef;

/**
 * @brief PID 输出整形调度配置。
 * @note  调度器只关心误差绝对值，误差单位由调用者自行保证一致。
 */
typedef struct
{
    float medium_enter_error;
    float medium_exit_error;
    float fine_enter_error;
    float fine_exit_error;

    float fast_filter_tau_s;
    float medium_filter_tau_s;
    float fine_filter_tau_s;

    float fast_slew_rate;
    float medium_slew_rate;
    float fine_slew_rate;
} PID_Output_Schedule_Config_TypeDef;

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
    float Friction_Compensation_out; // 摩擦补偿输出
    
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

    // ===== 10. 摩擦补偿（可选）=====
    bool friction_comp_enable; // 摩擦补偿使能
    float friction_fc; // 库仑摩擦补偿系数
    float friction_bv; // 粘性摩擦补偿系数
    float friction_w_eps; // 平滑参数，避免符号函数抖振
    bool friction_use_target_omega; // true=使用目标值作为omega，false=使用输入值作为omega

    // ===== 11. 输出整形（可选）=====
    bool output_filter_enable; // 输出低通使能
    float output_filter_tau_s; // 输出低通时间常数（秒）
    bool output_slew_enable;   // 输出斜率限制使能
    float output_slew_rate;    // 输出最大变化率（单位/秒）
    bool output_schedule_inited; // 输出调度状态是否已初始化
    PID_Output_Schedule_Mode_TypeDef output_schedule_mode; // 当前输出调度档位
    bool output_shaper_inited; // 输出整形状态是否已初始化
    float output_shaper_state; // 输出整形内部状态

    // ===== 12. 时间间隔 =====
    float dt; // 时间间隔（单位：秒）
} PID_TypeDef;
void PID_Init(PID_TypeDef *pid);

void PID_Differential_Enable(PID_TypeDef *pid,bool enable);

void PID_Deadband_Enable(PID_TypeDef *pid,bool enable,float deadband);

void PID_Friction_Compensation_Enable(PID_TypeDef *pid, bool enable, float Fc, float Bv, float w_eps, bool use_target_omega);

void PID_Output_Filter_Enable(PID_TypeDef *pid, bool enable, float tau_s);

/**
 * @brief 使能/禁用 PID 输出斜率限制。
 * @param pid PID 控制器指针
 * @param enable 是否使能
 * @param slew_rate 输出增大时的最大变化率（单位/秒）
 * @retval 无
 * @note  当前实现为非对称斜率限制：
 *        输出增大时按 slew_rate 限制，输出回收/反向时内部使用更快的释放速率，
 *        以减少输出端整形带来的相位滞后。
 */
void PID_Output_Slew_Enable(PID_TypeDef *pid, bool enable, float slew_rate);

/**
 * @brief 复位 PID 输出调度状态。
 * @param pid PID 控制器指针
 * @param mode 复位后的默认档位
 * @retval 无
 * @note  该接口只复位档位状态，不重置输出整形内部状态，
 *        以避免切换模式时引入额外输出突变。
 */
void PID_Output_Schedule_Reset(PID_TypeDef *pid, PID_Output_Schedule_Mode_TypeDef mode);

/**
 * @brief 按误差大小为 PID 输出整形选择 FAST / MEDIUM / FINE 档位。
 * @param pid PID 控制器指针
 * @param abs_error 当前误差绝对值
 * @param config 三档输出整形配置
 * @retval 无
 * @note  该接口内部带滞回切档逻辑，适合在每个控制周期调用一次。
 */
void PID_Output_Schedule_Apply(PID_TypeDef *pid, float abs_error, const PID_Output_Schedule_Config_TypeDef *config);

void PID_Output_Shaper_Reset(PID_TypeDef *pid, float init_output);

void PID_Integral_Separation_Enable(PID_TypeDef *pid,bool enable,float threshold_A,float threshold_B);

void PID_Target_Limit_Enable(PID_TypeDef *pid,bool enable,float min,float max);

void PID_Set_Parameters(PID_TypeDef *pid,float P,float I,float D,float FeedForward,float integral_min,float integral_max,float out_min,float out_max);

float PID_Calculate(PID_TypeDef *pid,float Input,float Target,float dt);

#endif /* __ALG_PID_H__ */
