#ifndef __DVC_MANIFOLD_H__
#define __DVC_MANIFOLD_H__

#include "drv_usb.h"
#include "alg_quaternion.h"
/**
 * @brief 视觉Manifold状态
 *
 */
enum Enum_Manifold_Status
{
    Manifold_Status_DISABLE = 0,
    Manifold_Status_ENABLE,
};

/**
 * @brief 视觉Manifold自家颜色
 *
 */
enum Enum_Manifold_Enemy_Color 
{
    Manifold_Enemy_Color_RED = 0,
    Manifold_Enemy_Color_BLUE,
};

/**
 * @brief 敌方机器人ID
 *
 */
enum Enum_Manifold_Enemy_ID 
{
    Manifold_Enemy_ID_NONE_0 = 0,
    Manifold_Enemy_ID_HERO_1,
    Manifold_Enemy_ID_ENGINEER_2,
    Manifold_Enemy_ID_INFANTRY_3,
    Manifold_Enemy_ID_INFANTRY_4,
    Manifold_Enemy_ID_INFANTRY_5,
    Manifold_Enemy_ID_SENTRY_7,
    Manifold_Enemy_ID_OUTPOST,
    Manifold_Enemy_ID_RUNE,
    Manifold_Enemy_ID_UNKNOWN,
};

/**
 * @brief 哨兵模式状态
 *
 */
enum Enum_Manifold_Sentry_Mode
{
    Manifold_Sentry_Mode_DISABLE = 0,  // 哨兵模式关闭
    Manifold_Sentry_Mode_ENABLE,       // 哨兵模式开启
};
/**
 * @brief 视觉Manifold给控制板的源数据
 *
 */
typedef struct 
{
    uint8_t Frame_Header;//帧头
    uint8_t Frame_Tail;//帧尾    
    euler_t Taget_Angle; //目标欧拉角
    uint8_t Shoot_Flag; //
    enum Enum_Manifold_Enemy_ID Enemy_ID;//敌方机器人ID
    uint16_t Confidence_Level;//置信度
}Manifold_UART_Rx_Data;

/**
 * @brief 控制板给视觉Manifold的源数据
 *
 */
typedef struct 
{
    uint8_t Frame_Header;//帧头
    uint8_t Frame_Tail;//帧尾
    euler_t Euler_Angle;//欧拉角
    enum Enum_Manifold_Sentry_Mode Sentry_Mode;//哨兵模式状态
}Manifold_UART_Tx_Data;

// 函数声明
void Manifold_Init(Manifold_UART_Tx_Data* data, uint8_t Frame_Header, uint8_t Frame_End, enum Enum_Manifold_Sentry_Mode Sentry_Mode);
/**
 * @brief manifold USB 接收回调函数
 * @note  非官方
 */
void Manifold_USB_Rx_Callback(uint8_t* Buf, uint32_t Len);
void Manifold_USB_SendData(Manifold_UART_Tx_Data *data , euler_t Euler);
#endif /*__DVC_MANIFOLD_H__*/
