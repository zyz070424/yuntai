#include "Gimbal.h"
#include "alg_pid.h"
#include "drv_usb.h"
#include "dvc_motor.h"
#include "alg_dwt.h"
#include "portmacro.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <math.h>
#define PI   3.1415926f
#define GIMBAL_CTRL_PERIOD_TICK     1
#define GIMBAL_CTRL_DT              0.001f
#define GIMBAL_MAHONY_KP            0.5f
#define GIMBAL_MAHONY_KI            0.001f

// IMU 实际 dt 估计参数（优先 DWT，失败回退 HAL tick）
#define GIMBAL_IMU_DT_DEFAULT_S     GIMBAL_CTRL_DT
#define GIMBAL_IMU_DT_MIN_S         0.0002f
#define GIMBAL_IMU_DT_MAX_S         0.0100f

// IMU 数据就绪中断开关：0=纯任务轮询，1=中断唤醒任务（中断里不读SPI）
#define GIMBAL_IMU_DRDY_ENABLE      0
// 中断模式下任务等待超时（tick），超时后回退一次轮询读取
#define GIMBAL_IMU_WAIT_TIMEOUT_TICK 2

// 机械限位（单位：度），用于保护线束
#define GIMBAL_PITCH_MIN_ANGLE     (-42.0f)
#define GIMBAL_PITCH_MAX_ANGLE     (42.0f)
#define GIMBAL_YAW_MIN_ANGLE       (-120.0f)
#define GIMBAL_YAW_MAX_ANGLE       (120.0f)

// 电机输出限幅（GM6020电压模式常用范围）
#define GIMBAL_MOTOR_CMD_LIMIT      10000.0f

// Yaw测试信号与串级整形参数
#define GIMBAL_YAW_TEST_FREQ_HZ              0.30f
#define GIMBAL_YAW_TEST_AMP_DEG              35.0f
#define GIMBAL_YAW_TARGET_SPEED_TAU_S        0.030f
#define GIMBAL_YAW_TARGET_SPEED_SLEW_RAD_S2  8.0f

// Yaw外环参数（角度环 -> 速度目标）
#define GIMBAL_YAW_ANGLE_KP                  0.12f
#define GIMBAL_YAW_ANGLE_KI                  0.1f
#define GIMBAL_YAW_ANGLE_KD                  0.00f
#define GIMBAL_YAW_ANGLE_OUT_LIMIT           4.0f
#define GIMBAL_YAW_ANGLE_I_LIMIT             1.5f
#define GIMBAL_YAW_ANGLE_DEADBAND_DEG        0.4f

// Yaw内环参数（速度环 -> 电机控制量）
#define GIMBAL_YAW_SPEED_KP                  2500.0f
#define GIMBAL_YAW_SPEED_KI                  700.0f
#define GIMBAL_YAW_SPEED_KD                  0.00f
#define GIMBAL_YAW_SPEED_I_LIMIT             2000.0f

// Pitch测试信号与串级整形参数（与Yaw同结构）
#define GIMBAL_PITCH_TEST_FREQ_HZ              0.60f
#define GIMBAL_PITCH_TEST_AMP_DEG              40.0f
#define GIMBAL_PITCH_TARGET_SPEED_TAU_S        0.020f
#define GIMBAL_PITCH_TARGET_SPEED_SLEW_RAD_S2  12.0f

// Pitch外环参数（角度环 -> 速度目标）
#define GIMBAL_PITCH_ANGLE_KP                  0.14f
#define GIMBAL_PITCH_ANGLE_KI                  0.1f
#define GIMBAL_PITCH_ANGLE_KD                  0.00f
#define GIMBAL_PITCH_ANGLE_OUT_LIMIT           4.5f
#define GIMBAL_PITCH_ANGLE_I_LIMIT             1.5f
#define GIMBAL_PITCH_ANGLE_DEADBAND_DEG        0.4f

// Pitch内环参数（速度环 -> 电机控制量）
#define GIMBAL_PITCH_SPEED_KP                  1100.0f
#define GIMBAL_PITCH_SPEED_KI                  280.0f
#define GIMBAL_PITCH_SPEED_KD                  0.00f
#define GIMBAL_PITCH_SPEED_I_LIMIT             2000.0f
// 运行时锁零掩码：支持按轴选择
#define GIMBAL_ZERO_LOCK_PITCH      (1 << 0)
#define GIMBAL_ZERO_LOCK_YAW        (1 << 1)
#define GIMBAL_ZERO_LOCK_BOTH       (GIMBAL_ZERO_LOCK_PITCH | GIMBAL_ZERO_LOCK_YAW)

Motor_TypeDef Gimbal_Motor_Pitch;
Motor_TypeDef Gimbal_Motor_Yaw;
imu_data_t Gimbal_IMU_Data;
euler_t Gimbal_Euler_Angle_to_send;
extern Manifold_UART_Rx_Data Rx_Data;
Manifold_UART_Tx_Data Tx_Data;

static float g_gimbal_pitch_target = 0.0f;
static float g_gimbal_yaw_target = 0.0f;
static float g_gimbal_pitch_zero = 0.0f;
static float g_gimbal_yaw_zero = 0.0f;
static uint8_t g_zero_locked_mask = 0;
#if GIMBAL_IMU_DRDY_ENABLE
static TaskHandle_t g_gimbal_imu_task_handle = NULL;
#endif
static alg_dwt_timebase_t g_gimbal_imu_timebase;
volatile float g_gimbal_imu_last_dt_s = GIMBAL_IMU_DT_DEFAULT_S;
volatile uint8_t g_gimbal_imu_last_dt_from_dwt = 0;
static uint8_t g_gimbal_yaw_continuous_inited = 0u;
static float g_gimbal_yaw_zero_raw_deg = 0.0f;
static float g_gimbal_yaw_last_rel_wrapped_deg = 0.0f;
static float g_gimbal_yaw_continuous_deg = 0.0f;
volatile float g_gimbal_yaw_raw_deg = 0.0f;
volatile float g_gimbal_yaw_send_deg = 0.0f;
float Target_Yaw = GIMBAL_YAW_TEST_AMP_DEG;
// 测试用：单轴正弦目标
float Target_Pitch = GIMBAL_PITCH_TEST_AMP_DEG;
static float g_gimbal_yaw_test_phase = 0.0f;
static float g_gimbal_pitch_test_phase = 0.0f;

/**
 * @brief   生成幅值周期翻转的阶跃信号
 * @param   elapsed_ms: 当前已运行时间（毫秒）
 * @param   flip_interval_ms: 每次翻转间隔（毫秒）
 * @param   amplitude: 阶跃幅值，输出将在 +amplitude 和 -amplitude 之间切换
 * @retval  当前时刻对应的翻转阶跃信号值
 * @note    使用示例：
 *          target = Gimbal_Generate_Flip_Step_Signal(gyro_test_time_ms, 1000u, 20.0f);
 *          这表示每 1000ms 翻转一次，在 +20 和 -20 之间切换。
 */
static float Gimbal_Generate_Flip_Step_Signal(uint32_t elapsed_ms,
                                              uint32_t flip_interval_ms,
                                              float amplitude)
{
    float amplitude_abs;
    uint32_t flip_count;

    if (isfinite(amplitude) == 0)
    {
        return 0.0f;
    }

    if (flip_interval_ms == 0u)
    {
        return 0.0f;
    }

    amplitude_abs = fabsf(amplitude);
    if (amplitude_abs <= 0.0f)
    {
        return 0.0f;
    }

    flip_count = elapsed_ms / flip_interval_ms;
    if ((flip_count & 1u) == 0u)
    {
        return amplitude_abs;
    }

    return -amplitude_abs;
}

/**
 * @brief   将角度限制到 [-180, 180) 区间
 * @param  angle_deg: 输入角度（deg）
 * @retval 区间化后的角度（deg）
 */
static float Gimbal_Wrap180(float angle_deg)
{
    while (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/**
 * @brief   重置yaw连续化状态（用于上电首次/链路恢复）
 * @retval 无
 */
static void Gimbal_Yaw_Continuous_Reset(void)
{
    g_gimbal_yaw_continuous_inited = 0u;
    g_gimbal_yaw_zero_raw_deg = 0.0f;
    g_gimbal_yaw_last_rel_wrapped_deg = 0.0f;
    g_gimbal_yaw_continuous_deg = 0.0f;
    g_gimbal_yaw_raw_deg = 0.0f;
    g_gimbal_yaw_send_deg = 0.0f;
}

/**
 * @brief   将姿态解算输出yaw转为“上电归零 + 连续角”
 * @param  raw_yaw_deg: 姿态解算原始yaw（通常在[-180,180)）
 * @retval 连续yaw（deg）
 */
static float Gimbal_Yaw_To_Continuous(float raw_yaw_deg)
{
    float yaw_rel_wrapped_deg;
    float dyaw_deg;

    if (isfinite(raw_yaw_deg) == 0)
    {
        return g_gimbal_yaw_continuous_deg;
    }

    g_gimbal_yaw_raw_deg = raw_yaw_deg;

    // 首次样本：锁定零偏，保证上电初值为0°
    if (g_gimbal_yaw_continuous_inited == 0u)
    {
        g_gimbal_yaw_zero_raw_deg = raw_yaw_deg;
        g_gimbal_yaw_last_rel_wrapped_deg = 0.0f;
        g_gimbal_yaw_continuous_deg = 0.0f;
        g_gimbal_yaw_continuous_inited = 1u;
        g_gimbal_yaw_send_deg = 0.0f;
        return 0.0f;
    }

    // 先做上电归零，再做区间化，避免+180/-180边界抖动
    yaw_rel_wrapped_deg = Gimbal_Wrap180(raw_yaw_deg - g_gimbal_yaw_zero_raw_deg);
    dyaw_deg = yaw_rel_wrapped_deg - g_gimbal_yaw_last_rel_wrapped_deg;

    if (dyaw_deg > 180.0f)
    {
        dyaw_deg -= 360.0f;
    }
    else if (dyaw_deg < -180.0f)
    {
        dyaw_deg += 360.0f;
    }

    g_gimbal_yaw_continuous_deg += dyaw_deg;
    g_gimbal_yaw_last_rel_wrapped_deg = yaw_rel_wrapped_deg;
    g_gimbal_yaw_send_deg = g_gimbal_yaw_continuous_deg;

    return g_gimbal_yaw_continuous_deg;
}
/**
 * @brief   CAN断联保护动作
 * @param   无
 * @retval  无
 * @note    这里做“软保护”：
 *          - 清积分，防止恢复时积分冲击
 *          - 目标角归零，避免断联期间目标继续漂移
 */
static void Gimbal_CAN_Offline_Protect(void)
{
    Gimbal_Motor_Pitch.PID[0].integral = 0.0f;
    Gimbal_Motor_Pitch.PID[1].integral = 0.0f;
    Gimbal_Motor_Yaw.PID[0].integral = 0.0f;
    Gimbal_Motor_Yaw.PID[1].integral = 0.0f;

    g_gimbal_pitch_target = 0.0f;
    g_gimbal_yaw_target = 0.0f;
   
}

/**
 * @brief   CAN恢复在线后的动作
 * @param   无
 * @retval  无
 * @note    当前恢复时不额外处理，预留扩展点
 */
static void Gimbal_CAN_Online_Protect(void)
{

}

/**
 * @brief   SPI断联保护动作
 * @param   无
 * @retval  无
 * @note    当前策略：将对外发送欧拉角清零，避免持续发送陈旧姿态
 */
static void Gimbal_SPI_Offline_Protect(void)
{
    Gimbal_Yaw_Continuous_Reset();
    Gimbal_Euler_Angle_to_send.roll = 0.0f;
    Gimbal_Euler_Angle_to_send.pitch = 0.0f;
    Gimbal_Euler_Angle_to_send.yaw = 0.0f;
}

/**
 * @brief   SPI恢复在线后的动作
 * @param   无
 * @retval  无
 */
static void Gimbal_SPI_Online_Protect(void)
{
    Gimbal_Yaw_Continuous_Reset();
}

/**
 * @brief   USB断联保护动作
 * @param   无
 * @retval  无
 * @note    清空视觉增量和关键控制字段，避免断联后沿用陈旧数据
 */
static void Gimbal_USB_Offline_Protect(void)
{
    //Rx_Data.Shoot_Flag = 0;
    Rx_Data.Taget_Pitch = 0.0f;
    Rx_Data.Taget_Yaw = 0.0f;
    //Rx_Data.Enemy_ID = Manifold_Enemy_ID_NONE_0;
    //Rx_Data.Confidence_Level = 0;
}

/**
 * @brief   USB恢复在线后的动作
 * @param   无
 * @retval  无
 * @note    当前恢复时不额外处理，预留扩展点
 */
static void Gimbal_USB_Online_Protect(void)
{
}

/**
 * @brief   云台角度限幅函数
 * @param  value: 输入角度值（单位：度）
 * @param  min_value: 最小角度值（单位：度）
 * @param  max_value: 最大角度值（单位：度）
 * @retval 限幅后的角度值（单位：度）
 */
static float Gimbal_Clamp(float value, float min_value, float max_value)
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

/**
 * @brief   清空某个电机所有已启用PID的运行状态（不改参数）
 * @param  motor: 电机对象
 * @retval 无
 */
static void Gimbal_PID_Clear_Runtime(Motor_TypeDef *motor)
{
    uint8_t i;

    if (motor == NULL)
    {
        return;
    }

    for (i = 0; i < motor->PID_Use_Count && i < 2; i++)
    {
        motor->PID[i].integral = 0.0f;
        motor->PID[i].error = 0.0f;
        motor->PID[i].prev_error = 0.0f;
        motor->PID[i].target = 0.0f;
        motor->PID[i].prev_target = 0.0f;
        motor->PID[i].output = 0.0f;
        motor->PID[i].output_shaper_state = 0.0f;
        motor->PID[i].output_shaper_inited = true;
    }
}

/**
 * @brief   判断指定轴的编码器反馈是否就绪
 * @param  axis_mask: GIMBAL_ZERO_LOCK_PITCH / GIMBAL_ZERO_LOCK_YAW / 组合
 * @retval 1=就绪，0=未就绪
 */
static uint8_t Gimbal_Is_Encoder_Ready(uint8_t axis_mask)
{
    if (((axis_mask & GIMBAL_ZERO_LOCK_PITCH) != 0) &&
        (Gimbal_Motor_Pitch.RxData.Encoder_Initialized == 0))
    {
        return 0;
    }

    if (((axis_mask & GIMBAL_ZERO_LOCK_YAW) != 0) &&
        (Gimbal_Motor_Yaw.RxData.Encoder_Initialized == 0))
    {
        return 0;
    }

    return 1;
}

/**
 * @brief   尝试按掩码锁零（可按轴选择）
 * @param  axis_mask: GIMBAL_ZERO_LOCK_PITCH / GIMBAL_ZERO_LOCK_YAW / 组合
 * @param  require_all_ready: 1=要求掩码内所有轴都ready才执行，0=ready哪个锁哪个
 * @retval 无
 */
static void Gimbal_Try_Lock_Zero(uint8_t axis_mask, uint8_t require_all_ready)
{
    uint8_t valid_mask = (uint8_t)(axis_mask & GIMBAL_ZERO_LOCK_BOTH);

    if (valid_mask == 0)
    {
        return;
    }

    if ((require_all_ready != 0) && (Gimbal_Is_Encoder_Ready(valid_mask) == 0))
    {
        return;
    }

    if (((valid_mask & GIMBAL_ZERO_LOCK_PITCH) != 0) &&
        ((g_zero_locked_mask & GIMBAL_ZERO_LOCK_PITCH) == 0) &&
        (Gimbal_Motor_Pitch.RxData.Encoder_Initialized != 0))
    {
        g_gimbal_pitch_zero = Gimbal_Motor_Pitch.RxData.Angle;
        g_gimbal_pitch_target = 0.0f;
        Gimbal_PID_Clear_Runtime(&Gimbal_Motor_Pitch);
        g_zero_locked_mask |= GIMBAL_ZERO_LOCK_PITCH;
    }

    if (((valid_mask & GIMBAL_ZERO_LOCK_YAW) != 0) &&
        ((g_zero_locked_mask & GIMBAL_ZERO_LOCK_YAW) == 0) &&
        (Gimbal_Motor_Yaw.RxData.Encoder_Initialized != 0))
    {
        g_gimbal_yaw_zero = Gimbal_Motor_Yaw.RxData.Angle;
        g_gimbal_yaw_target = 0.0f;
        Gimbal_PID_Clear_Runtime(&Gimbal_Motor_Yaw);
        g_zero_locked_mask |= GIMBAL_ZERO_LOCK_YAW;
    }
}

/**
 * @brief   判断指定轴是否已锁零
 * @param  axis_mask: GIMBAL_ZERO_LOCK_PITCH / GIMBAL_ZERO_LOCK_YAW / 组合
 * @retval 1=已全部锁零，0=未全部锁零
 */
static uint8_t Gimbal_Is_Zero_Locked(uint8_t axis_mask)
{
    uint8_t valid_mask = (uint8_t)(axis_mask & GIMBAL_ZERO_LOCK_BOTH);
    return ((g_zero_locked_mask & valid_mask) == valid_mask) ? 1 : 0;
}

/**
 * @brief   浮点数到16位有符号整数的转换（带限幅）
 * @param  value: 输入浮点数（范围：-32768.0f ~ 32767.0f）
 * @retval 转换后的16位有符号整数（范围：-32768 ~ 32767）
 */
static int16_t Gimbal_FloatToInt16_Sat(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)value;
}

/**
 * @brief   CAN离线时的输出转换函数
 * @param   value: 任意输入值
 * @retval  固定返回0，确保离线期间电机输出为0
 */
static int16_t Gimbal_Output_To_CAN_Zero(float value)
{
    (void)value;
    return 0;
}

/**
 * @brief   CAN在线时的输出转换函数
 * @param   value: PID输出（浮点）
 * @retval  转换后的16位控制量
 */
static int16_t Gimbal_Output_To_CAN_Normal(float value)
{
    return Gimbal_FloatToInt16_Sat(value);
}

/**
 * @brief   云台初始化函数
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Init(void* pramas)
{
    (void)pramas;

    // 初始化视觉通信
    Manifold_Init(&Tx_Data, 0xFE, 0xFF, Manifold_Sentry_Mode_DISABLE);

    // 启动电机所在CAN总线
    CAN_Start(&hcan2);

    // 初始化IMU
    if (BMI088_Init(&hspi1) != HAL_OK)
    {
        return;
    }
    
    // 俯仰电机：角度外环 + 速度内环
    Motor_Init(&Gimbal_Motor_Pitch, 4, GM6020_Voltage, &hcan2, DJI_Control_Method_Angle);
    // PID[1]: 角度外环，输出目标速度（小限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Pitch, 1,
                         GIMBAL_PITCH_ANGLE_KP, GIMBAL_PITCH_ANGLE_KI, GIMBAL_PITCH_ANGLE_KD, 0.00f,
                         -GIMBAL_PITCH_ANGLE_OUT_LIMIT, GIMBAL_PITCH_ANGLE_OUT_LIMIT,
                         -GIMBAL_PITCH_ANGLE_I_LIMIT, GIMBAL_PITCH_ANGLE_I_LIMIT);
    // PID[0]: 速度内环，输出最终电机控制量（大限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Pitch, 0,
                         GIMBAL_PITCH_SPEED_KP, GIMBAL_PITCH_SPEED_KI, GIMBAL_PITCH_SPEED_KD, 0.00f,
                         -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT,
                         -GIMBAL_PITCH_SPEED_I_LIMIT, GIMBAL_PITCH_SPEED_I_LIMIT);
    // 角度外环输出整形（放在PID层）：低通 + 斜率限制
    PID_Output_Filter_Enable(&Gimbal_Motor_Pitch.PID[1], true, GIMBAL_PITCH_TARGET_SPEED_TAU_S);
    PID_Output_Slew_Enable(&Gimbal_Motor_Pitch.PID[1], true, GIMBAL_PITCH_TARGET_SPEED_SLEW_RAD_S2);
    // 启用死区，避免小信号时电机抖动
    //PID_Deadband_Enable(&Gimbal_Motor_Pitch.PID[1], 1, GIMBAL_PITCH_ANGLE_DEADBAND_DEG);

    // 偏航电机：角度外环 + 速度内环
    Motor_Init(&Gimbal_Motor_Yaw, 2, GM6020_Voltage, &hcan2, DJI_Control_Method_Angle);
    // PID[1]: 角度外环，输出目标速度（小限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Yaw, 1,
                         GIMBAL_YAW_ANGLE_KP, GIMBAL_YAW_ANGLE_KI, GIMBAL_YAW_ANGLE_KD, 0.00f,
                         -GIMBAL_YAW_ANGLE_OUT_LIMIT, GIMBAL_YAW_ANGLE_OUT_LIMIT,
                         -GIMBAL_YAW_ANGLE_I_LIMIT, GIMBAL_YAW_ANGLE_I_LIMIT);
    // PID[0]: 速度内环，输出最终电机控制量（大限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Yaw, 0,
                         GIMBAL_YAW_SPEED_KP, GIMBAL_YAW_SPEED_KI, GIMBAL_YAW_SPEED_KD, 0.00f,
                         -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT,
                         -GIMBAL_YAW_SPEED_I_LIMIT, GIMBAL_YAW_SPEED_I_LIMIT);
    // 角度外环输出整形（放在PID层）：低通 + 斜率限制
    //PID_Output_Filter_Enable(&Gimbal_Motor_Yaw.PID[1], true, GIMBAL_YAW_TARGET_SPEED_TAU_S);
    //PID_Output_Slew_Enable(&Gimbal_Motor_Yaw.PID[1], true, GIMBAL_YAW_TARGET_SPEED_SLEW_RAD_S2);
    // 启用死区，避免小信号时电机抖动
   
    // 运行时参考系状态清零，后续在控制任务中按轴锁零
    g_gimbal_pitch_target = 0.0f;
    g_gimbal_yaw_target = 0.0f;
    g_gimbal_pitch_zero = 0.0f;
    g_gimbal_yaw_zero = 0.0f;
    g_zero_locked_mask = 0;
}



/**
 * @brief   云台双轴PID测试任务（1kHz）
 * @param  params: 无
 * @retval 无
 * @note    使用相对角反馈，便于串级PID联调
 */
void Gimbal_Motor_Control_ALL_Test(void* params)
{
    TickType_t time;
    uint8_t can_online;
    float pitch_output;
    float pitch_target_speed = 1.5;
    float yaw_output;
    float yaw_target_speed;
    static uint32_t cnt;
    static int16_t (* const can_output_map[2])(float) =
    {
        Gimbal_Output_To_CAN_Zero,
        Gimbal_Output_To_CAN_Normal
    };

    
    (void)params;

    time = xTaskGetTickCount();

    // 测试任务启动时重置本次运行参考系
    g_gimbal_pitch_target = 0.0f;
    g_gimbal_yaw_target = 0.0f;
    g_gimbal_pitch_zero = 0.0f;
    g_gimbal_yaw_zero = 0.0f;
    g_zero_locked_mask = 0;

    pitch_output = 0.0f;
    yaw_output = 0.0f;
    while (1)
    {
        

        // 1) 刷新电机反馈
        Motor_CAN_Data_Receive(&Gimbal_Motor_Pitch);
        Motor_CAN_Data_Receive(&Gimbal_Motor_Yaw);

       // Pitch使用IMU世界坐标，不做锁零门控；仅Yaw需要锁零
        if (Gimbal_Is_Zero_Locked(GIMBAL_ZERO_LOCK_YAW) == 0u)
        {
            Gimbal_Try_Lock_Zero(GIMBAL_ZERO_LOCK_YAW, 1u);
        }

        // 未完成Yaw锁零前，不进入闭环，保持零输出避免误动作
        if (Gimbal_Is_Zero_Locked(GIMBAL_ZERO_LOCK_YAW) == 0u)
        {
            can_online = CAN_Alive_IsOnline(&hcan2);
            Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, can_output_map[can_online](0.0f));
            Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, can_output_map[can_online](0.0f));
            vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
            continue;
        }
        
       // 相位累加构造连续正弦目标，避免计数器抖动/溢出导致相位跳变
        g_gimbal_yaw_test_phase += 2.0f * PI * GIMBAL_YAW_TEST_FREQ_HZ * GIMBAL_CTRL_DT;
        if (g_gimbal_yaw_test_phase >= 2.0f * PI)
        {
            g_gimbal_yaw_test_phase -= 2.0f * PI;
        }

        g_gimbal_pitch_test_phase += 2.0f * PI * GIMBAL_PITCH_TEST_FREQ_HZ * GIMBAL_CTRL_DT;
        if (g_gimbal_pitch_test_phase >= 2.0f * PI)
        {
            g_gimbal_pitch_test_phase -= 2.0f * PI;
        }
        
       
        g_gimbal_yaw_target = Target_Yaw * sinf(g_gimbal_yaw_test_phase);

        g_gimbal_pitch_target = Target_Pitch * sinf(g_gimbal_pitch_test_phase);
        //pitch_target_speed = Gimbal_Generate_Flip_Step_Signal(time,1000,1.5);
        // 6) 串级PID计算（测试模式：显式调用外环和内环）
        pitch_target_speed = Motor_PID_Calculate_Angle(&Gimbal_Motor_Pitch, g_gimbal_pitch_target, -Gimbal_Euler_Angle_to_send.pitch, GIMBAL_CTRL_DT);
        // cnt ++;
        // if(cnt %1000 == 0)
        // {
        //     pitch_target_speed = -pitch_target_speed;
        // }
        pitch_output = Motor_PID_Calculate_Speed(&Gimbal_Motor_Pitch, pitch_target_speed, Gimbal_Motor_Pitch.RxData.Speed, GIMBAL_CTRL_DT);
        
        yaw_target_speed = Motor_PID_Calculate_Angle(&Gimbal_Motor_Yaw, g_gimbal_yaw_target, Gimbal_Euler_Angle_to_send.yaw, GIMBAL_CTRL_DT);
        yaw_output = Motor_PID_Calculate_Speed(&Gimbal_Motor_Yaw, yaw_target_speed, Gimbal_Motor_Yaw.RxData.Speed, GIMBAL_CTRL_DT);
        
        //输出限幅
        pitch_output = Gimbal_Clamp(pitch_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);
        yaw_output = Gimbal_Clamp(yaw_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);

        can_online = CAN_Alive_IsOnline(&hcan2);
        Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, can_output_map[can_online](pitch_output));
        Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, can_output_map[can_online](yaw_output));

        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
}
/**
 * @brief   IMU数据就绪外部中断回调（仅做任务通知，不在中断中读SPI）
 * @param  GPIO_Pin: 触发中断的GPIO引脚
 * @note   非官方
 * @retval 无
 */
void Gimbal_IMU_EXTI_Callback(uint16_t GPIO_Pin)
{
#if GIMBAL_IMU_DRDY_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((GPIO_Pin == ACCEL_INT_Pin) || (GPIO_Pin == GYRO_INT_Pin))
    {
        if (g_gimbal_imu_task_handle != NULL)
        {
            vTaskNotifyGiveFromISR(g_gimbal_imu_task_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
#else
    (void)GPIO_Pin;
#endif
}

/**
 * @brief   云台欧拉角任务（1kHz）
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Euler(void *pramas)
{
    float imu_dt;
    euler_t euler_raw;

    (void)pramas;

    ALG_DWT_Timebase_Init(&g_gimbal_imu_timebase, GIMBAL_IMU_DT_DEFAULT_S);
    Gimbal_Yaw_Continuous_Reset();
    
#if GIMBAL_IMU_DRDY_ENABLE
    g_gimbal_imu_task_handle = xTaskGetCurrentTaskHandle();
    
#else
    TickType_t time;
    time = xTaskGetTickCount();
#endif

    while (1)
    {
#if GIMBAL_IMU_DRDY_ENABLE
        // 中断模式：等待IMU DRDY通知，超时则回退一次轮询，避免中断异常导致任务停摆
        (void)ulTaskNotifyTake(pdTRUE, GIMBAL_IMU_WAIT_TIMEOUT_TICK);
#else
        // 纯任务轮询模式：固定1kHz节拍
        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
#endif

        // 周期采集IMU并更新欧拉角，供视觉发送使用
        BMI088_ReadGyro(&hspi1, &Gimbal_IMU_Data);
        BMI088_ReadAccel(&hspi1, &Gimbal_IMU_Data);
        BMI088_ReadTemp(&hspi1, &Gimbal_IMU_Data);
        imu_dt = ALG_DWT_Timebase_GetDtS(&g_gimbal_imu_timebase,
                                         GIMBAL_IMU_DT_DEFAULT_S,
                                         GIMBAL_IMU_DT_MIN_S,
                                         GIMBAL_IMU_DT_MAX_S);
        g_gimbal_imu_last_dt_s = g_gimbal_imu_timebase.last_dt_s;
        g_gimbal_imu_last_dt_from_dwt = g_gimbal_imu_timebase.last_dt_from_dwt;
        Gimbal_IMU_Data.dt = imu_dt;
        euler_raw = BMI088_Complementary_Filter(&Gimbal_IMU_Data, imu_dt, GIMBAL_MAHONY_KP, GIMBAL_MAHONY_KI);
        euler_raw.yaw = Gimbal_Yaw_To_Continuous(euler_raw.yaw);
        Gimbal_Euler_Angle_to_send = euler_raw;
    }
}

/**
 * @brief   云台manifold 控制任务
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Manifold_Control(void *pramas)
{
    TickType_t time;

    (void)pramas;

    time = xTaskGetTickCount();

    while (1)
    {
        // 周期发送当前欧拉角到视觉
        //USB_SendString("Gimbal Euler Angles: ");
        Manifold_USB_SendData(&Tx_Data, Gimbal_Euler_Angle_to_send);
        vTaskDelayUntil(&time, 10);
    }
}

/**
 * @brief   云台任务
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Task(void* pramas)
{
    TickType_t time;
    uint16_t alive_check_div = 0;
    uint8_t can_online_changed;
    uint8_t spi_online_changed;
    uint8_t usb_online_changed;
   
    static void (* const can_link_action[2])(void) =
    {
        Gimbal_CAN_Offline_Protect,
        Gimbal_CAN_Online_Protect
    };
    static void (* const spi_link_action[2])(void) =
    {
        Gimbal_SPI_Offline_Protect,
        Gimbal_SPI_Online_Protect
    };
    static void (* const usb_link_action[2])(void) =
    {
        Gimbal_USB_Offline_Protect,
        Gimbal_USB_Online_Protect
    };

    (void)pramas;

    time = xTaskGetTickCount();

    while (1)
    {
        // 每100ms执行一次CAN/SPI/USB链路存活判断（Flag/Pre_Flag机制）
        if (++alive_check_div >= 100)
        {
            alive_check_div = 0;
            CAN_Alive_Check_100ms(&hcan2);
            SPI_Alive_Check_100ms();
            USB_Alive_Check_100ms();
        }

        // 若检测到CAN在线状态发生变化，在任务层执行对应保护动作
        if (CAN_Alive_TryConsumeChanged(&hcan2, &can_online_changed) != 0)
        {
            can_link_action[can_online_changed]();
        }

        // 若检测到SPI在线状态发生变化，在任务层执行对应保护动作
        if (SPI_Alive_TryConsumeChanged(&spi_online_changed) != 0)
        {
            spi_link_action[spi_online_changed]();
        }

        // 若检测到USB在线状态发生变化，在任务层执行对应保护动作
        if (USB_Alive_TryConsumeChanged(&usb_online_changed) != 0)
        {
            usb_link_action[usb_online_changed]();
        }
       
        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
}
