#include "Gimbal.h"

#define GIMBAL_CTRL_PERIOD_TICK     1
#define GIMBAL_CTRL_DT              0.001f

// IMU 数据就绪中断开关：0=纯任务轮询，1=中断唤醒任务（中断里不读SPI）
#define GIMBAL_IMU_DRDY_ENABLE      1
// 中断模式下任务等待超时（tick），超时后回退一次轮询读取
#define GIMBAL_IMU_WAIT_TIMEOUT_TICK 2

// 机械限位（单位：度），用于保护线束
#define GIMBAL_PITCH_MIN_ANGLE     (-15.0f)
#define GIMBAL_PITCH_MAX_ANGLE     (30.0f)
#define GIMBAL_YAW_MIN_ANGLE       (-120.0f)
#define GIMBAL_YAW_MAX_ANGLE       (120.0f)

// 电机输出限幅（GM6020电压模式常用范围）
#define GIMBAL_MOTOR_CMD_LIMIT      12000.0f

// 单次视觉增量限幅，防止异常包导致目标突变
#define GIMBAL_VISION_INC_LIMIT     2.0f

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
static uint8_t g_pitch_target_inited = 0;
static uint8_t g_yaw_target_inited = 0;
static TaskHandle_t g_gimbal_imu_task_handle = NULL;
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
 * @brief   云台初始化函数
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Init(void* pramas)
{
    (void)pramas;

    // 初始化视觉通信
    Manifold_Init(&Tx_Data, 0xA5, 0x5A, Manifold_Sentry_Mode_DISABLE);

    // 启动电机所在CAN总线
    CAN_Start(&hcan1);

    // 初始化IMU
    if (BMI088_Init(&hspi1) != HAL_OK)
    {
        return;
    }

    // 俯仰电机：角度外环 + 速度内环
    Motor_Init(&Gimbal_Motor_Pitch, 1, GM6020_Voltage, &hcan1, DJI_Control_Method_Angle);
    Motor_Set_PID_Params(&Gimbal_Motor_Pitch, 0, 0.20f, 0.00f, 0.00f, 0.00f, -6.0f, 6.0f, -1.0f, 1.0f);
    Motor_Set_PID_Params(&Gimbal_Motor_Pitch, 1, 1800.0f, 20.0f, 0.0f, 0.0f, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT, -3000.0f, 3000.0f);

    // 偏航电机：角度外环 + 速度内环
    Motor_Init(&Gimbal_Motor_Yaw, 2, GM6020_Voltage, &hcan1, DJI_Control_Method_Angle);
    Motor_Set_PID_Params(&Gimbal_Motor_Yaw, 0, 0.20f, 0.00f, 0.00f, 0.00f, -6.0f, 6.0f, -1.0f, 1.0f);
    Motor_Set_PID_Params(&Gimbal_Motor_Yaw, 1, 1800.0f, 20.0f, 0.0f, 0.0f, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT, -3000.0f, 3000.0f);

    // 清零目标和零点，等待电机首帧反馈后锁定
    g_gimbal_pitch_target = 0.0f;
    g_gimbal_yaw_target = 0.0f;
    g_gimbal_pitch_zero = 0.0f;
    g_gimbal_yaw_zero = 0.0f;
    g_pitch_target_inited = 0;
    g_yaw_target_inited = 0;
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

    (void)pramas;

    time = xTaskGetTickCount();

    while (1)
    {
        // 先刷新电机反馈
        Motor_CAN_Data_Receive(&Gimbal_Motor_Pitch);
        Motor_CAN_Data_Receive(&Gimbal_Motor_Yaw);

        // 首帧反馈后记录零点，后续使用相对角度闭环
        if ((g_pitch_target_inited == 0) && (Gimbal_Motor_Pitch.RxData.Encoder_Initialized != 0))
        {
            g_gimbal_pitch_zero = Gimbal_Motor_Pitch.RxData.Angle;
            g_gimbal_pitch_target = 0.0f;
            g_pitch_target_inited = 1;
        }

        if ((g_yaw_target_inited == 0) && (Gimbal_Motor_Yaw.RxData.Encoder_Initialized != 0))
        {
            g_gimbal_yaw_zero = Gimbal_Motor_Yaw.RxData.Angle;
            g_gimbal_yaw_target = 0.0f;
            g_yaw_target_inited = 1;
        }

        // 视觉给的是角度增量，本周期按增量叠加目标角
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

        // 下发电机控制
        Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, Gimbal_FloatToInt16_Sat(pitch_output));
        Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, Gimbal_FloatToInt16_Sat(yaw_output));

        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
}
/**
 * @brief   IMU数据就绪外部中断回调（仅做任务通知，不在中断中读SPI）
 * @param  GPIO_Pin: 触发中断的GPIO引脚
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
    TickType_t time;

    (void)pramas;

    g_gimbal_imu_task_handle = xTaskGetCurrentTaskHandle();
    time = xTaskGetTickCount();

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

    (void)pramas;

    time = xTaskGetTickCount();

    while (1)
    {
        if (Tx_Data.Sentry_Mode)
        {
            // 后续在此补充哨兵搜索逻辑
        }

        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
}



