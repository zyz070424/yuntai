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
 * @brief   状态灯模式定义
 * @note    当前实现CAN/SPI/USB三路灯语：
 *          - All_Online：慢速心跳
 *          - CAN_Offline：快速闪烁（高优先级）
 *          - SPI_Offline：中速闪烁
 *          - USB_Offline：长亮短灭
 */
enum Enum_Status_LED_Mode
{
    Status_LED_Mode_All_Online = 0,
    Status_LED_Mode_CAN_Offline,
    Status_LED_Mode_SPI_Offline,
    Status_LED_Mode_USB_Offline,
};

/**
 * @brief   状态灯闪烁参数
 * @param   on_time_ms: 亮灯时长（ms）
 * @param   off_time_ms: 灭灯时长（ms）
 */
typedef struct
{
    uint16_t on_time_ms;
    uint16_t off_time_ms;
} Status_LED_Pattern_TypeDef;

// 默认上电先按CAN离线模式显示，直到检测到链路在线
static volatile enum Enum_Status_LED_Mode g_status_led_mode = Status_LED_Mode_CAN_Offline;
// 当前闪烁周期内的相位计数（ms）
static uint16_t g_status_led_phase_ms = 0;
// 四种模式对应闪烁节奏
static const Status_LED_Pattern_TypeDef g_status_led_pattern[4] =
{
    {50, 950},   // 全在线：心跳
    {120, 120},  // CAN离线：快闪
    {300, 300},  // SPI离线：中速闪烁
    {700, 200},  // USB离线：长亮短灭
};

/**
 * @brief   状态灯底层输出接口（弱定义）
 * @param   state: 0=灭灯，1=亮灯
 * @retval  无
 * @note    你可以在其他文件里实现同名强符号，绑定到实际GPIO口
 */
__weak void Gimbal_Status_LED_Write(uint8_t state)
{
    (void)state;
}

/**
 * @brief   设置状态灯当前模式
 * @param   mode: 目标模式
 * @retval  无
 */
static void Gimbal_Status_LED_Set_Mode(enum Enum_Status_LED_Mode mode)
{
    g_status_led_mode = mode;
}

/**
 * @brief   根据CAN/SPI/USB在线状态选择灯语模式（表驱动）
 * @param   can_online: CAN是否在线（0/1）
 * @param   spi_online: SPI是否在线（0/1）
 * @param   usb_online: USB是否在线（0/1）
 * @retval  状态灯模式
 * @note    优先级：CAN离线 > SPI离线 > USB离线 > 全在线
 */
static enum Enum_Status_LED_Mode Gimbal_Status_LED_Select_Mode(uint8_t can_online, uint8_t spi_online, uint8_t usb_online)
{
    // bit0: CAN离线，bit1: SPI离线，bit2: USB离线
    uint8_t fault_mask = (uint8_t)((can_online == 0) | ((spi_online == 0) << 1) | ((usb_online == 0) << 2));
    static const enum Enum_Status_LED_Mode mode_map[8] =
    {
        Status_LED_Mode_All_Online,  // 000
        Status_LED_Mode_CAN_Offline, // 001
        Status_LED_Mode_SPI_Offline, // 010
        Status_LED_Mode_CAN_Offline, // 011
        Status_LED_Mode_USB_Offline, // 100
        Status_LED_Mode_CAN_Offline, // 101
        Status_LED_Mode_SPI_Offline, // 110
        Status_LED_Mode_CAN_Offline  // 111
    };

    return mode_map[fault_mask];
}

/**
 * @brief   10ms周期更新一次状态灯输出
 * @param   无
 * @retval  无
 * @note    本函数不阻塞、不延时，只按相位推进输出
 */
static void Gimbal_Status_LED_Update_10ms(void)
{
    Status_LED_Pattern_TypeDef pattern = g_status_led_pattern[g_status_led_mode];
    uint16_t period_ms = (uint16_t)(pattern.on_time_ms + pattern.off_time_ms);
    uint16_t pos_ms = 0;

    // 防御性保护：防止后续误改造成周期为0
    if (period_ms == 0)
    {
        Gimbal_Status_LED_Write(0);
        return;
    }

    pos_ms = (uint16_t)(g_status_led_phase_ms % period_ms);
    Gimbal_Status_LED_Write((uint8_t)(pos_ms < pattern.on_time_ms));

    g_status_led_phase_ms = (uint16_t)(g_status_led_phase_ms + 10);
    if (g_status_led_phase_ms >= period_ms)
    {
        g_status_led_phase_ms = 0;
    }
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

        // 根据链路在线状态选择输出策略：
        // 离线 -> 固定输出0；在线 -> 正常输出PID结果
        can_online = CAN_Alive_IsOnline(&hcan1);
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
    uint16_t led_update_div = 0;
    uint8_t can_online_changed;
    uint8_t spi_online_changed;
    uint8_t usb_online_changed;
    uint8_t can_online;
    uint8_t spi_online;
    uint8_t usb_online;
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
            CAN_Alive_Check_100ms(&hcan1);
            SPI_Alive_Check_100ms();
            USB_Alive_Check_100ms();
        }

        // 若检测到CAN在线状态发生变化，在任务层执行对应保护动作
        if (CAN_Alive_TryConsumeChanged(&hcan1, &can_online_changed) != 0)
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

        // 每个周期按当前链路状态选择灯语模式
        can_online = CAN_Alive_IsOnline(&hcan1);
        spi_online = SPI_Alive_IsOnline();
        usb_online = USB_Alive_IsOnline();
        Gimbal_Status_LED_Set_Mode(Gimbal_Status_LED_Select_Mode(can_online, spi_online, usb_online));

        // 每10ms刷新一次状态灯输出
        if (++led_update_div >= 10)
        {
            led_update_div = 0;
            Gimbal_Status_LED_Update_10ms();
        }

        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
}
