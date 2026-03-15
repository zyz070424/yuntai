#include "alg_pid.h"
/**
 * @brief 初始化 PID 控制器
 * @param pid: PID 控制器指针
 */
void PID_Init(PID_TypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->FeedForward = 0.0f;

    pid->P_out = 0.0f;
    pid->I_out = 0.0f;
    pid->D_out = 0.0f;
    pid->FeedForward_out = 0.0f;

    pid->target = 0.0f;
    pid->prev_target = 0.0f;
    pid->output = 0.0f;
    pid->Input = 0.0f;

    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;

    pid->out_min = 0.0f;
    pid->out_max = 0.0f;

    pid->integral_min = 0.0f;
    pid->integral_max = 0.0f;

    pid->target_limit_enable = false;
    pid->target_limit_min = 0.0f;
    pid->target_limit_max = 0.0f;

    pid->integral_separation_enable = false;
    pid->integral_separation_threshold_A = 0.0f;
    pid->integral_separation_threshold_B = 0.0f;

    pid->differential_enable = false;

    pid->deadband_enable = false;
    pid->deadband = 0.0f;

    pid->dt = 0.0f;
}
/**
 * @brief 使能/禁用目标值限幅
 * @param pid: PID 控制器指针
 * @param enable: 是否使能限幅
 * @param min: 目标值最小值
 * @param max: 目标值最大值
 */
void PID_Target_Limit_Enable(PID_TypeDef *pid, bool enable, float min, float max)
{
    if (pid == NULL)
    {
        return;
    }

    pid->target_limit_enable = enable;
    pid->target_limit_min = min;
    pid->target_limit_max = max;
}
/**
 * @brief 使能/禁用积分分离
 * @param pid: PID 控制器指针
 * @param enable: 是否使能积分分离
 * @param threshold_A: 积分分离阈值A
 * @param threshold_B: 积分分离阈值B
 */
void PID_Integral_Separation_Enable(PID_TypeDef *pid, bool enable, float threshold_A, float threshold_B)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral_separation_enable = enable;
    pid->integral_separation_threshold_A = threshold_A;
    pid->integral_separation_threshold_B = threshold_B;
}
/**
 * @brief 使能/禁用微分项
 * @param pid: PID 控制器指针
 * @param enable: 是否使能微分项
 */
void PID_Differential_Enable(PID_TypeDef *pid, bool enable)
{
    if (pid == NULL)
    {
        return;
    }

    pid->differential_enable = enable;
}
/**
 * @brief 使能/禁用死区
 * @param pid: PID 控制器指针
 * @param enable: 是否使能死区
 * @param deadband: 死区值
 */
void PID_Deadband_Enable(PID_TypeDef *pid, bool enable, float deadband)
{
    if (pid == NULL)
    {
        return;
    }

    pid->deadband_enable = enable;
    pid->deadband = deadband;
}
/**
 * @brief 设置 PID 控制器参数
 * @param pid: PID 控制器指针
 * @param P: 比例系数
 * @param I: 积分系数
 * @param D: 微分系数
 * @param FeedForward: 前馈系数
 * @param integral_min: 积分项限幅最小值
 * @param integral_max: 积分项限幅最大值
 * @param out_min: 输出限幅最小值
 * @param out_max: 输出限幅最大值
 */
void PID_Set_Parameters(PID_TypeDef *pid, float P, float I, float D, float FeedForward,
                        float integral_min, float integral_max, float out_min, float out_max)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Kp = P;
    pid->Ki = I;
    pid->Kd = D;
    pid->FeedForward = FeedForward;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->out_min = out_min;
    pid->out_max = out_max;
}
/**
 * @brief 计算 PID 控制器输出
 * @param pid: PID 控制器指针
 * @param Input: 输入值
 * @param Target: 目标值
 * @param dt: 时间间隔
 * @return float: PID 控制器输出值
 */
float PID_Calculate(PID_TypeDef *pid, float Input, float Target, float dt)
{
    const float dt_min = 1e-6f;
    float integral_coef = 1.0f;
    // 检查 PID 控制器指针是否为空
    if (pid == NULL)
    {
        return 0.0f;
    }
    // 检查时间间隔是否过小，若过小时，将其设为最小允许值
    if (dt < dt_min)
    {
        dt = dt_min;
    }
    
    pid->dt = dt;
    pid->Input = Input;
    pid->target = Target;
    // 检查目标值是否超出限幅范围，若超出则进行限幅
    if (pid->target_limit_enable)
    {
        if (pid->target < pid->target_limit_min)
        {
            pid->target = pid->target_limit_min;
        }
        else if (pid->target > pid->target_limit_max)
        {
            pid->target = pid->target_limit_max;
        }
    }

    pid->error = pid->target - pid->Input;
    // 检查误差是否在死区内，若在则输出为0
    if (pid->deadband_enable && fabsf(pid->error) <= pid->deadband)
    {
        pid->output = 0.0f;
        pid->P_out = 0.0f;
        pid->I_out = 0.0f;
        pid->D_out = 0.0f;
        pid->FeedForward_out = 0.0f;
        pid->prev_error = pid->error;
        pid->prev_target = pid->target;
        return 0.0f;
    }
    // 计算比例项输出
    pid->P_out = pid->Kp * pid->error;
    // 检查是否启用积分分离
    if (pid->integral_separation_enable)
    {
        float A = pid->integral_separation_threshold_A;
        float B = pid->integral_separation_threshold_B;
        float abs_err = fabsf(pid->error);

        if (A <= B)
        {
            A = B + 1e-6f;
        }

        if (abs_err >= A)
        {
            integral_coef = 0.0f;
        }
        else if (abs_err > B)
        {
            integral_coef = (A - abs_err) / (A - B);
        }
    }
    // 计算积分项输出
    pid->integral += pid->error * pid->dt * integral_coef;
    // 检查积分项是否超出限幅范围，若超出则进行限幅
    if (pid->integral < pid->integral_min)
    {
        pid->integral = pid->integral_min;
    }
    else if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    // 计算积分项输出
    pid->I_out = pid->Ki * pid->integral;
    // 检查是否使用微分先行
    if (pid->differential_enable)
    {
        pid->D_out = pid->Kd * (-(pid->target - pid->prev_target) / pid->dt);
    }
    else
    {
        pid->D_out = pid->Kd * ((pid->error - pid->prev_error) / pid->dt);
    }
    // 计算前馈项输出
    pid->FeedForward_out = pid->FeedForward * pid->target;
    // 计算输出值
    pid->output = pid->P_out + pid->I_out + pid->D_out + pid->FeedForward_out;
    // 检查输出是否超出限幅范围，若超出则进行限幅
    if (pid->output < pid->out_min)
    {
        pid->output = pid->out_min;
    }
    else if (pid->output > pid->out_max)
    {
        pid->output = pid->out_max;
    }
    // 更新前一个误差和目标值
    pid->prev_error = pid->error;
    pid->prev_target = pid->target;

    return pid->output;
}
