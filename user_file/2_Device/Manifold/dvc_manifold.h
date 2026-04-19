#ifndef __DVC_MANIFOLD_H__
#define __DVC_MANIFOLD_H__

#include "drv_usb.h"
#include "alg_quaternion.h"
#include <stdint.h>

/**
 * @brief 视觉发给电控的接收帧长度。
 * @note  协议固定为 [Frame_Header][Yaw][Pitch][Target_Valid][Frame_Tail]。
 */
#define MANIFOLD_USB_RX_FRAME_LEN     (1u + sizeof(float) + sizeof(float) + 1u + 1u)
/**
 * @brief 电控发给视觉的发送帧长度。
 * @note  本次改造只调整接收协议，发送协议仍保持 [Frame_Header][Yaw][Pitch][Frame_Tail]。
 */
#define MANIFOLD_USB_TX_FRAME_LEN     (1u + sizeof(float) + sizeof(float) + 1u)
#define MANIFOLD_USB_RX_DEBUG_RAW_MAX USB_BUFFER_SIZE
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


/**
 * @brief 视觉发给电控的目标数据缓存。
 * @note  线上协议字节布局固定为 [Header][Yaw][Pitch][Target_Valid][Tail]，
 *        解析时必须按协议偏移读取，不能依赖结构体内存布局。
 */
typedef struct
{
    uint8_t Frame_Header; // 帧头
    float Taget_Yaw;      // 目标偏航角（绝对角）
    float Taget_Pitch;    // 目标俯仰角（绝对角）
    uint8_t Target_Valid; // 目标是否有效：0=无目标，非0=有目标
    uint8_t Frame_Tail;   // 帧尾
    // euler_t Taget_Angle; //目标欧拉角
    // uint8_t Shoot_Flag; //
    // enum Enum_Manifold_Enemy_ID Enemy_ID;//敌方机器人ID
    // uint16_t Confidence_Level;//置信度
} Manifold_UART_Rx_Data;

/**
 * @brief 控制板给视觉Manifold的源数据
 *
 */
/**
 * @brief 电控发给视觉的姿态数据缓存。
 * @note  发送协议仍保持 [Header][Yaw][Pitch][Tail]，不携带 Target_Valid。
 */
typedef struct
{
    uint8_t Frame_Header; // 帧头
    float Yaw;            // 偏航角
    float Pitch;          // 俯仰角
    uint8_t Frame_Tail;   // 帧尾
} Manifold_UART_Tx_Data;

// 函数声明
extern volatile uint8_t Manifold_USB_Tx_Debug_Frame[MANIFOLD_USB_TX_FRAME_LEN];
extern volatile uint16_t Manifold_USB_Tx_Debug_Len;
extern volatile uint8_t Manifold_USB_Rx_Debug_Raw[MANIFOLD_USB_RX_DEBUG_RAW_MAX];
extern volatile uint16_t Manifold_USB_Rx_Debug_Raw_Len;
extern volatile uint8_t Manifold_USB_Rx_Debug_Frame[MANIFOLD_USB_RX_FRAME_LEN];
extern volatile uint16_t Manifold_USB_Rx_Debug_Frame_Len;
extern volatile uint32_t Manifold_USB_Rx_Frame_Seq;

void Manifold_Init(Manifold_UART_Tx_Data* data, uint8_t Frame_Header, uint8_t Frame_End, enum Enum_Manifold_Sentry_Mode Sentry_Mode);
void Manifold_Clear_Target(void);
/**
 * @brief manifold USB 接收回调函数
 * @note  非官方
 */
void Manifold_USB_Rx_Callback(uint8_t* Buf, uint32_t Len);
void Manifold_USB_SendData(Manifold_UART_Tx_Data *data , euler_t Euler);
#endif /*__DVC_MANIFOLD_H__*/
