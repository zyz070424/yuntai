#include "dvc_motor.h"
#include <stdint.h>
#include <string.h>

typedef struct
{
    uint8_t frame_0x200[8];
    uint8_t frame_0x1FF[8];
    uint8_t frame_0x2FF[8];
    uint8_t frame_0x1FE[8];
    uint8_t frame_0x2FE[8];
} Motor_CAN_Tx_Cache_TypeDef;

static Motor_CAN_Tx_Cache_TypeDef Motor_CAN1_Tx_Cache = {0};
static Motor_CAN_Tx_Cache_TypeDef Motor_CAN2_Tx_Cache = {0};

/**
 * @brief 获取对应CAN总线的发送缓存
 * @param can CAN句柄
 * @retval 发送缓存指针，失败返回NULL
 */
static Motor_CAN_Tx_Cache_TypeDef *Motor_Get_Tx_Cache(CAN_HandleTypeDef *can)
{
    if (can == NULL)
    {
        return NULL;
    }

    if (can->Instance == CAN1)
    {
        return &Motor_CAN1_Tx_Cache;
    }

    if (can->Instance == CAN2)
    {
        return &Motor_CAN2_Tx_Cache;
    }

    return NULL;
}

/**
 * @brief 获取某个控制帧ID对应的8字节发送缓存
 * @param can CAN句柄
 * @param send_id 控制帧ID
 * @retval 缓冲区指针，失败返回NULL
 */
static uint8_t *Motor_Get_Tx_Frame_Buffer(CAN_HandleTypeDef *can, uint32_t send_id)
{
    Motor_CAN_Tx_Cache_TypeDef *cache = Motor_Get_Tx_Cache(can);

    if (cache == NULL)
    {
        return NULL;
    }

    switch (send_id)
    {
        case 0x200:
            return cache->frame_0x200;
        case 0x1FF:
            return cache->frame_0x1FF;
        case 0x2FF:
            return cache->frame_0x2FF;
        case 0x1FE:
            return cache->frame_0x1FE;
        case 0x2FE:
            return cache->frame_0x2FE;
        default:
            return NULL;
    }
}

/**
 * @brief 更新控制帧中某个电机槽位并发送，可以有效防止频繁更新tx缓存
 * @param motor 电机对象
 * @param send_id 控制帧ID
 * @param byte_index 当前电机对应的高字节索引（0/2/4/6）
 * @param data 电机控制量
 */
static void Motor_Update_Frame_And_Send(Motor_TypeDef *motor, uint32_t send_id, uint8_t byte_index, int16_t data)
{
    uint8_t *tx_data;

    if (motor == NULL || motor->can == NULL)
    {
        return;
    }

    if (byte_index > 6)
    {
        return;
    }

    tx_data = Motor_Get_Tx_Frame_Buffer(motor->can, send_id);
    if (tx_data == NULL)
    {
        return;
    }

    // 只更新当前电机在控制帧中的两个字节，其它电机字节保持上次值
    tx_data[byte_index] = (uint8_t)(data >> 8);
    tx_data[byte_index + 1] = (uint8_t)data;

    CAN_Send(motor->can, send_id, tx_data);
}

/**
 * @brief 初始化电机
 * @param motor 电机结构体指针
 * @param ID 电机ID
 * @param type 电机类型
 * @param can CAN句柄
 * @param method 控制方法
 */
void Motor_Init(Motor_TypeDef *motor, uint8_t ID, enum Motor_DJI_type type,
                CAN_HandleTypeDef *can, enum Motor_DJI_Control_Method method)
{
    if (motor == NULL)
    {
        return;
    }

    // CAN 启动统一在系统初始化阶段完成，这里仅保存句柄
    motor->can = can;
    motor->ID = ID;
    motor->type = type;
    motor->method = method;

    // 清空接收状态，保证首包按初始化流程处理
    memset(&motor->RxData, 0, sizeof(motor->RxData));

    // 根据控制方法动态分配PID控制器
    switch (method)
    {
        case DJI_Control_Method_Speed:
            motor->PID_Use_Count = 1;
            // 初始限幅设为0，后续通过Motor_Set_PID_Params设置
            PID_Init(&motor->PID[0]);
            break;

        case DJI_Control_Method_Angle:
            motor->PID_Use_Count = 2;
            // 外环角度PID
            PID_Init(&motor->PID[0]);
            // 内环速度PID
            PID_Init(&motor->PID[1]);
            break;

        default:
            motor->PID_Use_Count = 0;
            break;
    }
}

/**
 * @brief 设置电机PID参数（系数和限幅）
 * @param motor 电机结构体指针
 * @param pid_index PID索引（0:外环/单级, 1:内环）
 * @param p 比例系数
 * @param i 积分系数
 * @param d 微分系数
 * @param feedforward 前馈系数
 * @param out_min 输出最小值（通常为负的电流限幅）
 * @param out_max 输出最大值
 * @param integral_min 积分最小值
 * @param integral_max 积分最大值
 */
void Motor_Set_PID_Params(Motor_TypeDef *motor, uint8_t pid_index,
                          float p, float i, float d, float feedforward,
                          float out_min, float out_max,
                          float integral_min, float integral_max)
{
    PID_Set_Parameters(&motor->PID[pid_index], p, i, d, feedforward, integral_min, integral_max, out_min, out_max);
}


/**
 * @brief 电机PID计算（自动选择单级或级联模式）
 * @param motor 电机结构体指针
 * @param target 目标值（角度或速度，取决于模式）
 * @param feedback_angle 当前角度反馈/也就是真实值，可以转欧拉角（用于角度模式的外环，速度模式下可传入任意值）
 * @param dt 时间间隔（秒）
 * @return 计算后的输出值（期望电流/转矩）
 */
float Motor_PID_Calculate(Motor_TypeDef *motor, float target, float feedback_angle, float dt)
{
    if (motor == NULL)
        return 0.0f;

    switch (motor->method)
    {
        case DJI_Control_Method_Speed:
            // 单级PID：直接速度控制（反馈为电机当前速度）
            return PID_Calculate(&motor->PID[0], (float)motor->RxData.Speed, target, dt);

        case DJI_Control_Method_Angle:
        {
            // 级联PID：外环角度PID -> 内环速度PID
            // 外环：目标角度 vs 当前角度，输出为目标速度
            float target_speed = PID_Calculate(&motor->PID[0], feedback_angle, target, dt);
            // 内环：目标速度 vs 当前速度，输出为最终电流
            return PID_Calculate(&motor->PID[1], (float)motor->RxData.Speed, target_speed, dt);
        }

        default:
            return 0.0f;
    }
}


/**
 * @brief 处理GM6020原始数据（字节解析）
 * @param motor 电机结构体指针
 * @param data 接收数据指针（8字节）
 */
static void Motor_GM6020_Data_Process(Motor_TypeDef *motor, uint8_t *data)
{
    int32_t delta_encoder;
    uint16_t Encoder_Angle = (uint16_t)data[0] << 8 | data[1];

    motor->RxData.Speed       = (int16_t)((uint16_t)data[2] << 8 | data[3]) * RPM_TO_RADS;
    motor->RxData.Torque      = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    motor->RxData.Temperature = data[6];

    // 首包只对齐编码器零点，不做圈数累计，防止上电第一帧跳圈
    if (motor->RxData.Encoder_Initialized == 0)
    {
        motor->RxData.Last_encoder_angle = Encoder_Angle;
        motor->RxData.Total_Round = 0;
        motor->RxData.Total_Encode = Encoder_Angle;
        motor->RxData.Angle = (float)motor->RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round;
        motor->RxData.Encoder_Initialized = 1;
        return;
    }

    delta_encoder = Encoder_Angle - motor->RxData.Last_encoder_angle;
    if(delta_encoder > Encoder_Num_Per_Round / 2)
    {
        motor->RxData.Total_Round--;
    }
    else if(delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        motor->RxData.Total_Round++;
    }
    motor->RxData.Last_encoder_angle = Encoder_Angle;
    motor->RxData.Total_Encode = motor->RxData.Total_Round * Encoder_Num_Per_Round + Encoder_Angle;
    motor->RxData.Angle = (float)motor->RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round;
}

/**
 * @brief 处理M3508原始数据（字节解析）
 * @param motor 电机结构体指针
 * @param data 接收数据指针（8字节）
 */
static void Motor_M3508_Data_Process(Motor_TypeDef *motor, uint8_t *data)
{
    int32_t delta_encoder;
    uint16_t Encoder_Angle = (uint16_t)data[0] << 8 | data[1];

    motor->RxData.Speed       = (int16_t)((uint16_t)data[2] << 8 | data[3]) * RPM_TO_RADS / M2508_Gearbox_Rate;
    motor->RxData.Torque      = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    motor->RxData.Temperature = data[6];

    // 首包只对齐编码器零点，不做圈数累计，防止上电第一帧跳圈
    if (motor->RxData.Encoder_Initialized == 0)
    {
        motor->RxData.Last_encoder_angle = Encoder_Angle;
        motor->RxData.Total_Round = 0;
        motor->RxData.Total_Encode = Encoder_Angle;
        motor->RxData.Angle = (float)motor->RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round / M2508_Gearbox_Rate;
        motor->RxData.Encoder_Initialized = 1;
        return;
    }

    delta_encoder = Encoder_Angle - motor->RxData.Last_encoder_angle;
    if(delta_encoder > Encoder_Num_Per_Round / 2)
    {
        motor->RxData.Total_Round--;
    }
    else if(delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        motor->RxData.Total_Round++;
    }
    motor->RxData.Last_encoder_angle = Encoder_Angle;
    motor->RxData.Total_Encode = motor->RxData.Total_Round * Encoder_Num_Per_Round + Encoder_Angle;
    motor->RxData.Angle = (float)motor->RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round / M2508_Gearbox_Rate;
}

void Motor_Angle_Limit(Motor_TypeDef *motor,float min,float max)
{
    PID_Target_Limit_Enable(&motor->PID[0],true,min,max);
    if(motor->PID[0].target < min)
    {
       motor->PID[0].target = min;
    }
    else if(motor->PID[0].target > max)
    {
       motor->PID[0].target = max;
    }
}

/**
 * @brief 接收电机CAN数据（按ID从CAN双缓冲中提取）
 * @param motor 电机结构体指针
 */
void Motor_CAN_Data_Receive(Motor_TypeDef *motor)
{
    CAN_RX_MESSAGE RxBuffer;
    uint32_t feedback_id;

    if (motor == NULL || motor->can == NULL)
    {
        return;
    }

    if (motor->ID < 1 || motor->ID > 8)
    {
        return;
    }

    switch (motor->type)
    {
        case M3508:
            feedback_id = 0x200 + motor->ID;
            break;

        case GM6020_Voltage:
        case GM6020_Current:
            // 你确认ID从1开始：ID1 -> 0x205
            feedback_id = 0x204 + motor->ID;
            break;

        default:
            return;
    }

    // 仅提取当前电机对应ID的数据，不会误消费其他电机帧
    // 周期任务中一次尽量读空该ID，减少反馈滞后
    while (CAN_ReadMessage_By_StdId(motor->can, feedback_id, &RxBuffer) == HAL_OK)
    {
        switch (motor->type)
        {
            case M3508:
                Motor_M3508_Data_Process(motor, RxBuffer.rx_data);
                break;

            case GM6020_Voltage:
            case GM6020_Current:
                Motor_GM6020_Data_Process(motor, RxBuffer.rx_data);
                break;

            default:
                return;
        }
    }
}

/**
 * @brief 发送电机CAN数据（根据电机类型和ID）
 * @param motor 电机结构体指针
 * @param data 要发送的数据（16位）
 */
void Motor_Send_CAN_Data(Motor_TypeDef *motor, int16_t data)
{
    if (motor == NULL || motor->can == NULL)
    {
        return;
    }

    if (motor->ID < 1 || motor->ID > 8)
    {
        return;
    }

    switch (motor->type)
    {
        case M3508:
            switch (motor->ID)
            {
                case 1:
                    Motor_Update_Frame_And_Send(motor, 0x200, 0, data);
                    break;
                case 2:
                    Motor_Update_Frame_And_Send(motor, 0x200, 2, data);
                    break;
                case 3:
                    Motor_Update_Frame_And_Send(motor, 0x200, 4, data);
                    break;
                case 4:
                    Motor_Update_Frame_And_Send(motor, 0x200, 6, data);
                    break;
                case 5:
                    Motor_Update_Frame_And_Send(motor, 0x1FF, 0, data);
                    break;
                case 6:
                    Motor_Update_Frame_And_Send(motor, 0x1FF, 2, data);
                    break;
                case 7:
                    Motor_Update_Frame_And_Send(motor, 0x1FF, 4, data);
                    break;
                case 8:
                    Motor_Update_Frame_And_Send(motor, 0x1FF, 6, data);
                    break;
                default:
                    break;
            }
            break;

        case GM6020_Voltage:
            switch (motor->ID)
            {
                case 1:
                    Motor_Update_Frame_And_Send(motor, 0x1FF, 0, data);
                    break;
                case 2:
                    Motor_Update_Frame_And_Send(motor, 0x1FF, 2, data);
                    break;
                case 3:
                    Motor_Update_Frame_And_Send(motor, 0x1FF, 4, data);
                    break;
                case 4:
                    Motor_Update_Frame_And_Send(motor, 0x1FF, 6, data);
                    break;
                case 5:
                    Motor_Update_Frame_And_Send(motor, 0x2FF, 0, data);
                    break;
                case 6:
                    Motor_Update_Frame_And_Send(motor, 0x2FF, 2, data);
                    break;
                case 7:
                    Motor_Update_Frame_And_Send(motor, 0x2FF, 4, data);
                    break;
                case 8:
                    Motor_Update_Frame_And_Send(motor, 0x2FF, 6, data);
                    break;
                default:
                    break;
            }
            break;

        case GM6020_Current:
            switch (motor->ID)
            {
                case 1:
                    Motor_Update_Frame_And_Send(motor, 0x1FE, 0, data);
                    break;
                case 2:
                    Motor_Update_Frame_And_Send(motor, 0x1FE, 2, data);
                    break;
                case 3:
                    Motor_Update_Frame_And_Send(motor, 0x1FE, 4, data);
                    break;
                case 4:
                    Motor_Update_Frame_And_Send(motor, 0x1FE, 6, data);
                    break;
                case 5:
                    Motor_Update_Frame_And_Send(motor, 0x2FE, 0, data);
                    break;
                case 6:
                    Motor_Update_Frame_And_Send(motor, 0x2FE, 2, data);
                    break;
                case 7:
                    Motor_Update_Frame_And_Send(motor, 0x2FE, 4, data);
                    break;
                case 8:
                    Motor_Update_Frame_And_Send(motor, 0x2FE, 6, data);
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
}
