#ifndef DRV_USB_H__
#define DRV_USB_H__

#include "main.h"
#include "usbd_cdc_if.h"
#include <stdint.h>

#define USB_DATA_Send_MAX 256
#define USB_BUFFER_SIZE   256

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

    // 接收时间戳
    uint64_t Rx_Time_Stamp;
};

void USB_Init(USB_Callback callback);
uint8_t USB_SendData(const uint8_t *data, uint16_t len);
uint8_t USB_SendString(const char *str);
void USB_Rx_Callback(uint8_t *buf, uint32_t len);

#endif /* DRV_USB_H__ */