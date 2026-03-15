#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "drv_can.h"
#include "alg_pid.h"
#include "stm32f4xx_hal_can.h"
#include <stdint.h>


#define Encoder_Num_Per_Round 8192

#define M2508_Gearbox_Rate   268.0f/17.0f

#define RPM_TO_RADS 0.104719755f

enum Motor_DJI_Control_Method
{
    DJI_Control_Method_Angle = 0,
    DJI_Control_Method_Speed = 1,
};
enum Motor_DJI_type
{
    GM6020_Current,
    GM6020_Voltage,
    M3508,
};

typedef struct
{
    float Angle;
    float Speed;
    int16_t Torque;
    uint8_t Temperature;

    int32_t Total_Encode;
    uint16_t Last_encoder_angle;
    int32_t  Total_Round;
    // 首包标志：防止上电第一帧出现跨圈误判
    uint8_t Encoder_Initialized;

}Motor_DataTypeDef;

typedef struct
{
    // PID控制器
    PID_TypeDef PID[2];
    uint8_t PID_Use_Count;
    // 电机ID
    uint8_t ID;
    // 电机类型
    enum Motor_DJI_type type;
    // CAN句柄
    CAN_HandleTypeDef* can;

    // 控制方法
    enum Motor_DJI_Control_Method method;
    //电机数据
    Motor_DataTypeDef RxData;


} Motor_TypeDef;

void Motor_Init(Motor_TypeDef *motor, uint8_t ID, enum Motor_DJI_type type,
                CAN_HandleTypeDef *can, enum Motor_DJI_Control_Method method);
void Motor_Set_PID_Params(Motor_TypeDef *motor, uint8_t pid_index,
                          float p, float i, float d, float feedforward,
                          float out_min, float out_max,
                          float integral_min, float integral_max);
float Motor_PID_Calculate(Motor_TypeDef *motor, float target, float feedback_angle, float dt);
void Motor_CAN_Data_Receive(Motor_TypeDef *motor);
void Motor_Send_CAN_Data(Motor_TypeDef *motor, int16_t data);
#endif /* __MOTOR_H__ */
