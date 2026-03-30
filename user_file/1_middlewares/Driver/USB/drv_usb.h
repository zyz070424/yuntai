#ifndef DRV_USB_H__
#define DRV_USB_H__

#include "main.h"
#include "usbd_cdc_if.h"
#include <stdint.h>

#define USB_DATA_Send_MAX 256
#define USB_BUFFER_SIZE   256
#define USB_TX_MIN_INTERVAL_TICK_DEFAULT 1
#define USB_TX_BUSY_TIMEOUT_TICK 20

typedef void (*USB_Callback)(uint8_t *buffer, uint32_t length);

/**
 * @brief USB通信处理结构体
 */
struct Struct_USB_Manage_Object
{
    USB_Callback Callback_Function;

    // 双缓冲接收缓存
    uint8_t Rx_Buffer_0[USB_BUFFER_SIZE];
    uint8_t *Rx_Buffer_Active;
    uint8_t *Rx_Buffer_Ready;

    // USB发送状态：0=空闲 1=发送中
    volatile uint8_t Tx_Busy;
    // 两次发送最小间隔（tick）
    uint16_t Tx_Min_Interval_Tick;
    // 上一次发送成功发起的系统tick
    uint32_t Tx_Last_Transmit_Tick;
    // 上一次进入发送忙状态的系统tick（用于忙状态超时恢复）
    uint32_t Tx_Busy_Start_Tick;

    // ========= 通信存活检测（Alive Guard） =========
    // 在“收到有效上位机数据”时自增，用于判断最近100ms是否有USB接收
    volatile uint32_t Alive_Flag;
    // 100ms检查时保存上次Alive_Flag，与当前比较判定在线/离线
    uint32_t Alive_Pre_Flag;
    // 当前在线状态：0=离线，1=在线
    volatile uint8_t Alive_Online;
    // 状态变化标志：0=未变化，1=状态变化（供任务层消费）
    volatile uint8_t Alive_Changed;
};

extern struct Struct_USB_Manage_Object USB_Manage_Object;

void USB_Init(USB_Callback callback);
uint8_t USB_SendData(const uint8_t *data, uint16_t len);
uint8_t USB_SendString(const char *str);
// 设置USB最小发送间隔（tick），传0会按1tick处理
void USB_Set_Tx_Min_Interval(uint16_t interval_tick);
/**
 * @brief USB 接收回调函数
 * @note  非官方
 */
void USB_Rx_Callback(uint8_t *buf, uint32_t len);
/**
 * @brief USB 发送完成回调函数
 * @note  非官方
 */
void USB_TxCplt_Callback(void);

// 100ms周期检查一次USB链路是否在线（Flag是否增长）
void USB_Alive_Check_100ms(void);
// 获取当前USB在线状态：0=离线，1=在线
uint8_t USB_Alive_IsOnline(void);
// 任务层消费“在线状态变化事件”，有变化返回1并输出当前online值
uint8_t USB_Alive_TryConsumeChanged(uint8_t *online);

#endif /* DRV_USB_H__ */
