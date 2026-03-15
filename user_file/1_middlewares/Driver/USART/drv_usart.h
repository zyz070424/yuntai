#ifndef DRV_USART_H__
#define DRV_USART_H__

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#define UART_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256

typedef void (*UART_Callback)(uint8_t *buffer, uint16_t length);

struct Struct_UART_Manage_Object
{
    // UART句柄
    UART_HandleTypeDef *huart;
    UART_Callback Callback_Function;

    // 双缓冲适配的接收缓冲区
    uint8_t Rx_Buffer_0[UART_BUFFER_SIZE];
    uint8_t Rx_Buffer_1[UART_BUFFER_SIZE];

    // 正在接收的缓冲区
    uint8_t *Rx_Buffer_Active;
    // 接收完毕的缓冲区
    uint8_t *Rx_Buffer_Ready;

    // DMA发送缓存（避免上层传入栈内存导致DMA访问无效地址）
    uint8_t Tx_Buffer[UART_TX_BUFFER_SIZE];
    volatile uint8_t Tx_Busy;
};

extern struct Struct_UART_Manage_Object UART1_Manage_Object;
extern struct Struct_UART_Manage_Object UART5_Manage_Object;
extern struct Struct_UART_Manage_Object UART6_Manage_Object;

void USART_Init(UART_HandleTypeDef *huart, UART_Callback callback);
HAL_StatusTypeDef USART_SendData(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len);

#endif /* DRV_USART_H__ */
