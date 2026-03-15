#ifndef __VOFA_PLUS_H__
#define __VOFA_PLUS_H__

#include "drv_usart.h"
#include <stdint.h>
#include <string.h>
// VOFA 协议的尾包
void VOFA_Init(UART_HandleTypeDef *huart);
void VOFA_SendData_JustFloat(UART_HandleTypeDef *huart, float *data, uint16_t count);
void VOFA_SendData_FireWater(UART_HandleTypeDef *huart,char *data);
#endif /* __VOFA_PLUS_H__ */