#include "Gimbal.h"
#include "dvc_motor.h"
#include "portmacro.h"
#include <stdint.h>

#define GIMBAL_CTRL_PERIOD_TICK     1
#define GIMBAL_CTRL_DT              0.001f

// IMU 数据就绪中断开关：0=纯任务轮询，1=中断唤醒任务（中断里不读SPI）
#define GIMBAL_IMU_DRDY_ENABLE      1
// 中断模式下任务等待超时（tick），超时后回退一次轮询读取
#define GIMBAL_IMU_WAIT_TIMEOUT_TICK 2

// 机械限位（单位：度），用于保护线束
#define GIMBAL_PITCH_MIN_ANGLE     (-42.0f)
#define GIMBAL_PITCH_MAX_ANGLE     (42.0f)
#define GIMBAL_YAW_MIN_ANGLE       (-120.0f)
#define GIMBAL_YAW_MAX_ANGLE       (120.0f)

// 电机输出限幅（GM6020电压模式常用范围）
#define GIMBAL_MOTOR_CMD_LIMIT      3000.0f

// 单次视觉增量限幅，防止异常包导致目标突变
#define GIMBAL_VISION_INC_LIMIT     2.0f

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
static TaskHandle_t g_gimbal_imu_task_handle = NULL;
float Target = 60;
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
}

/**
 * @brief   USB断联保护动作
 * @param   无
 * @retval  无
 * @note    清空视觉增量和关键控制字段，避免断联后沿用陈旧数据
 */
static void Gimbal_USB_Offline_Protect(void)
{
    Rx_Data.Shoot_Flag = 0;
    Rx_Data.Gimbal_Pitch_Angle_Increment = 0.0f;
    Rx_Data.Gimbal_Yaw_Angle_Increment = 0.0f;
    Rx_Data.Gimbal_Pitch_Omega_FeedForward = 0.0f;
    Rx_Data.Gimbal_Yaw_Omega_FeedForward = 0.0f;
    Rx_Data.Enemy_ID = Manifold_Enemy_ID_NONE_0;
    Rx_Data.Confidence_Level = 0;
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
    Motor_Set_PID_Params(&Gimbal_Motor_Pitch, 1, 0.00f, 0.00f, 0.00f, 0.00f, -2.0f, 2.0f, -0.5f, 0.5f);
    // PID[0]: 速度内环，输出最终电机控制量（大限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Pitch, 0, 0.00f, 0.00f, 0.00f, 0.00f, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT, -3000.0f,  3000.0f);

    // 偏航电机：角度外环 + 速度内环
    Motor_Init(&Gimbal_Motor_Yaw, 2, GM6020_Voltage, &hcan2, DJI_Control_Method_Angle);
    // PID[1]: 角度外环，输出目标速度（小限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Yaw, 1, 0.00f, 0.00f, 0.00f, 0.00f, -2.0f, 2.0f, -0.5f, 0.5f);
    // PID[0]: 速度内环，输出最终电机控制量（大限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Yaw, 0, 0.00f, 0.00f, 0.00f, 0.00f, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT, -3000.0f,  3000.0f);

    // 运行时参考系状态清零，后续在控制任务中按轴锁零
    g_gimbal_pitch_target = 0.0f;
    g_gimbal_yaw_target = 0.0f;
    g_gimbal_pitch_zero = 0.0f;
    g_gimbal_yaw_zero = 0.0f;
    g_zero_locked_mask = 0;
}

// 云台电机控制任务（1kHz）
/**
 * @brief   云台电机控制任务（1kHz）
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Motor_Control_test(void* pramas)
{
    TickType_t time;
    float pitch_inc;
    float yaw_inc;
    float pitch_feedback;
    float yaw_feedback;
    float pitch_output;
    float yaw_output;
    uint8_t can_online;
    static int16_t (* const can_output_map[2])(float) =
    {
        Gimbal_Output_To_CAN_Zero,
        Gimbal_Output_To_CAN_Normal
    };

    (void)pramas;

    time = xTaskGetTickCount();

    while (1)
    {
        // 先刷新电机反馈
        Motor_CAN_Data_Receive(&Gimbal_Motor_Pitch);
        Motor_CAN_Data_Receive(&Gimbal_Motor_Yaw);

        // 默认两轴同步锁零；若你只想锁单轴，可改成 GIMBAL_ZERO_LOCK_PITCH / YAW
        if (Gimbal_Is_Zero_Locked(GIMBAL_ZERO_LOCK_BOTH) == 0u)
        {
            Gimbal_Try_Lock_Zero(GIMBAL_ZERO_LOCK_BOTH, 1u);
        }

        // 未完成锁零前，不进入闭环，保持零输出避免误动作
        if (Gimbal_Is_Zero_Locked(GIMBAL_ZERO_LOCK_BOTH) == 0u)
        {
            can_online = CAN_Alive_IsOnline(&hcan2);
            Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, can_output_map[can_online](0.0f));
            Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, can_output_map[can_online](0.0f));
            vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
            continue;
        }

        //视觉给的是角度增量，本周期按增量叠加目标角
        pitch_inc = Gimbal_Clamp(Rx_Data.Gimbal_Pitch_Angle_Increment, -GIMBAL_VISION_INC_LIMIT, GIMBAL_VISION_INC_LIMIT);
        yaw_inc = Gimbal_Clamp(Rx_Data.Gimbal_Yaw_Angle_Increment, -GIMBAL_VISION_INC_LIMIT, GIMBAL_VISION_INC_LIMIT);

        g_gimbal_pitch_target += pitch_inc;
        g_gimbal_yaw_target += yaw_inc;

        // 软件限位，防止扯线
        g_gimbal_pitch_target = Gimbal_Clamp(g_gimbal_pitch_target, GIMBAL_PITCH_MIN_ANGLE, GIMBAL_PITCH_MAX_ANGLE);
        g_gimbal_yaw_target = Gimbal_Clamp(g_gimbal_yaw_target, GIMBAL_YAW_MIN_ANGLE, GIMBAL_YAW_MAX_ANGLE);

        // 相对角度反馈
        pitch_feedback = Gimbal_Motor_Pitch.RxData.Angle - g_gimbal_pitch_zero;
        yaw_feedback = Gimbal_Motor_Yaw.RxData.Angle - g_gimbal_yaw_zero;

        // PID计算
        pitch_output = Motor_PID_Calculate(&Gimbal_Motor_Pitch, g_gimbal_pitch_target, pitch_feedback, GIMBAL_CTRL_DT);
        yaw_output = Motor_PID_Calculate(&Gimbal_Motor_Yaw, g_gimbal_yaw_target, yaw_feedback, GIMBAL_CTRL_DT);

        pitch_output = Gimbal_Clamp(pitch_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);
        yaw_output = Gimbal_Clamp(yaw_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);

        // 根据链路在线状态选择输出策略：
        // 离线 -> 固定输出0；在线 -> 正常输出PID结果
        can_online = CAN_Alive_IsOnline(&hcan2);
        Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, can_output_map[can_online](pitch_output));
        Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, can_output_map[can_online](yaw_output));
        
        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
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
    float pitch_feedback;
    float pitch_output;
    float pitch_target_speed;
    float yaw_output;
    float yaw_feedback;
    float yaw_target_speed;

    
    (void)params;

    time = xTaskGetTickCount();

    // 测试任务启动时重置本次运行参考系
    g_gimbal_pitch_target = 0.0f;
    g_gimbal_yaw_target = 0.0f;
    g_gimbal_pitch_zero = 0.0f;
    g_gimbal_yaw_zero = 0.0f;
    g_zero_locked_mask = 0;

    pitch_feedback = 0.0f;
    yaw_feedback = 0.0f;
    pitch_output = 0.0f;
    yaw_output = 0.0f;
    uint32_t cnt = 0;
    while (1)
    {
        

        // 1) 刷新电机反馈
        Motor_CAN_Data_Receive(&Gimbal_Motor_Pitch);
        Motor_CAN_Data_Receive(&Gimbal_Motor_Yaw);

        // 2) 按轴一次性锁零（调PID时观察相对角更直观）
        if (((g_zero_locked_mask & GIMBAL_ZERO_LOCK_PITCH) == 0) &&
            (Gimbal_Motor_Pitch.RxData.Encoder_Initialized != 0))
        {
            g_gimbal_pitch_zero = Gimbal_Motor_Pitch.RxData.Angle;
            g_gimbal_pitch_target = 0.0f;
            Gimbal_Motor_Pitch.PID[0].integral = 0.0f;
            Gimbal_Motor_Pitch.PID[1].integral = 0.0f;
            g_zero_locked_mask |= GIMBAL_ZERO_LOCK_PITCH;
        }
        if (((g_zero_locked_mask & GIMBAL_ZERO_LOCK_YAW) == 0) &&
            (Gimbal_Motor_Yaw.RxData.Encoder_Initialized != 0))
        {
            g_gimbal_yaw_zero = Gimbal_Motor_Yaw.RxData.Angle;
            g_gimbal_yaw_target = 0.0f;
            Gimbal_Motor_Yaw.PID[0].integral = 0.0f;
            Gimbal_Motor_Yaw.PID[1].integral = 0.0f;
            g_zero_locked_mask |= GIMBAL_ZERO_LOCK_YAW;
        }

        can_online = CAN_Alive_IsOnline(&hcan2);

        // 3) 未完成双轴锁零前不进闭环，避免目标/反馈参考系不一致
        if ((g_zero_locked_mask & GIMBAL_ZERO_LOCK_BOTH) != GIMBAL_ZERO_LOCK_BOTH)
        {
            Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, Gimbal_Output_To_CAN_Zero(0.0f));
            Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, Gimbal_Output_To_CAN_Zero(0.0f));
            vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
            continue;
        }
        //累加
        cnt++;
        //产生方波信号
        if(cnt / 5000)
        {
            Target = -Target;
            g_gimbal_yaw_target = Target;
            cnt = 0;
        }
        Gimbal_Motor_Pitch.RxData.Angle = (Gimbal_Motor_Pitch.RxData.Angle - g_gimbal_pitch_zero);
        pitch_feedback = Gimbal_Motor_Pitch.RxData.Angle;
        Gimbal_Motor_Yaw.RxData.Angle = (Gimbal_Motor_Yaw.RxData.Angle - g_gimbal_yaw_zero);
        yaw_feedback = Gimbal_Motor_Yaw.RxData.Angle;
        

        // 6) 串级PID计算（测试模式：显式调用外环和内环）
        pitch_target_speed = Motor_PID_Calculate_Angle(&Gimbal_Motor_Pitch, g_gimbal_pitch_target, pitch_feedback, GIMBAL_CTRL_DT);
        pitch_output = Motor_PID_Calculate_Speed(&Gimbal_Motor_Pitch, pitch_target_speed, Gimbal_Motor_Pitch.RxData.Speed, GIMBAL_CTRL_DT);
        
        yaw_target_speed = Motor_PID_Calculate_Angle(&Gimbal_Motor_Yaw, g_gimbal_yaw_target, yaw_feedback, GIMBAL_CTRL_DT);
        yaw_output = Motor_PID_Calculate_Speed(&Gimbal_Motor_Yaw, yaw_target_speed, Gimbal_Motor_Yaw.RxData.Speed, GIMBAL_CTRL_DT);
        
        //输出限幅
        pitch_output = Gimbal_Clamp(pitch_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);
        yaw_output = Gimbal_Clamp(yaw_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);

        if (can_online != 0)
        {
            Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, Gimbal_Output_To_CAN_Normal(pitch_output));
        }
        else
        {
            Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, Gimbal_Output_To_CAN_Zero(0.0f));
        }   
        if (can_online != 0)
        {
            Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, Gimbal_Output_To_CAN_Normal(yaw_output));
        }
        else
        {
            Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, Gimbal_Output_To_CAN_Zero(0.0f));
        }
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
    (void)pramas;
    
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
        Gimbal_Euler_Angle_to_send = BMI088_Complementary_Filter(&Gimbal_IMU_Data, GIMBAL_CTRL_DT, 0.5f, 0.0f);
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
