#include "dvc_vofa.h"

static char tail[4] = {0x00, 0x00, 0x80, 0x7f};

/**
 * @brief  VOFA 接收回调函数
 * @param  data: 接收数据缓冲区指针
 * @param  len: 接收数据长度
 * @note   非官方
 * @retval 无
 */
void VOFA_Callback( uint8_t *data, uint16_t len)
{   
    
}


/**
 * @brief  VOFA 初始化函数
 * @param  huart: UART 句柄指针
 * @retval 无
 */
void VOFA_Init(UART_HandleTypeDef *huart)
{
    USART_Init(huart, VOFA_Callback);
}
/**
 * @brief  VOFA 发送单精度浮点数数据函数
 * @param  huart: UART 句柄指针
 * @param  data: 待发送的单精度浮点数数据指针
 * @param  count: 待发送的单精度浮点数数据数量
 * @retval 无
 */
void VOFA_SendData_JustFloat(UART_HandleTypeDef *huart, float *data, uint16_t count)
{
    uint8_t buffer[count * 4 + 4];
    for (uint16_t i = 0; i < count; i++)
    {
        memcpy(buffer + i * 4, data + i, 4);
    }
    memcpy(buffer + count * 4, tail, 4);
    USART_SendData(huart, buffer, count * 4 + 4);
    
}
/**
 * @brief  VOFA 发送字符串数据函数
 * @param  huart: UART 句柄指针
 * @param  data: 待发送的字符串数据指针
 * @retval 无
 */
void VOFA_SendData_FireWater(UART_HandleTypeDef *huart,char *data)
{
    uint16_t len = strlen(data);
    uint8_t buffer[len + 2];
    memcpy(buffer, data, len);
    buffer[len] = 0x0D;
    buffer[len + 1] = 0x0A;
    USART_SendData(huart, buffer, len + 2);
}






