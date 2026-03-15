#include "drv_usart.h"
#include <string.h>

struct Struct_UART_Manage_Object UART1_Manage_Object = {0};
struct Struct_UART_Manage_Object UART5_Manage_Object = {0};
struct Struct_UART_Manage_Object UART6_Manage_Object = {0};

/**
 * @brief 根据UART句柄获取管理对象
 * @param huart UART句柄
 * @retval 管理对象指针，失败返回NULL
 */
static struct Struct_UART_Manage_Object *USART_Get_Manage_Object(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return NULL;
    }

    if (huart->Instance == USART1)
    {
        return &UART1_Manage_Object;
    }

    if (huart->Instance == UART5)
    {
        return &UART5_Manage_Object;
    }

    if (huart->Instance == USART6)
    {
        return &UART6_Manage_Object;
    }

    return NULL;
}

/**
 * @brief 进入临界区（兼容调度器未启动阶段）
 */
static uint32_t USART_Enter_Critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief 退出临界区
 */
static void USART_Exit_Critical(uint32_t primask)
{
    if (primask == 0)
    {
        __enable_irq();
    }
}

/**
 * @brief 启动ReceiveToIdle接收（有DMA用DMA，无DMA用中断）
 * @param manage UART管理对象
 * @retval HAL状态码
 */
static HAL_StatusTypeDef USART_Start_Rx(struct Struct_UART_Manage_Object *manage)
{
    HAL_StatusTypeDef ret;

    if (manage == NULL || manage->huart == NULL || manage->Rx_Buffer_Active == NULL)
    {
        return HAL_ERROR;
    }

    if (manage->huart->hdmarx != NULL)
    {
        ret = HAL_UARTEx_ReceiveToIdle_DMA(manage->huart, manage->Rx_Buffer_Active, UART_BUFFER_SIZE);
        if (ret != HAL_OK)
        {
            return ret;
        }

        // 不需要半满中断，减少无效中断频率
        __HAL_DMA_DISABLE_IT(manage->huart->hdmarx, DMA_IT_HT);
        return HAL_OK;
    }

    return HAL_UARTEx_ReceiveToIdle_IT(manage->huart, manage->Rx_Buffer_Active, UART_BUFFER_SIZE);
}

/*
 * @brief  USART 初始化函数
 * @param  huart: UART 句柄指针
 * @param  callback: 接收完成回调函数指针
 * @retval 无
 */
void USART_Init(UART_HandleTypeDef *huart, UART_Callback callback)
{
    struct Struct_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);

    if (manage == NULL)
    {
        return;
    }

    manage->huart = huart;
    manage->Callback_Function = callback;
    manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
    manage->Rx_Buffer_Ready = manage->Rx_Buffer_1;
    manage->Tx_Busy = 0;

    (void)USART_Start_Rx(manage);
}

/*
 * @brief  USART 重新初始化函数（用于错误恢复）
 * @param  huart: UART 句柄指针
 * @retval 无
 */
static void UART_Reinit(UART_HandleTypeDef *huart)
{
    struct Struct_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);

    if (manage == NULL)
    {
        return;
    }

    manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
    manage->Rx_Buffer_Ready = manage->Rx_Buffer_1;
    manage->Tx_Busy = 0;

    (void)USART_Start_Rx(manage);
}

/*
 * @brief  USART 发送数据函数（DMA）
 * @param  huart: UART 句柄指针
 * @param  data: 待发送的数据指针
 * @param  len: 待发送的数据长度
 * @retval HAL状态码
 */
HAL_StatusTypeDef USART_SendData(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len)
{
    struct Struct_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);
    HAL_StatusTypeDef ret;
    uint32_t primask;

    if (manage == NULL || manage->huart == NULL || data == NULL)
    {
        return HAL_ERROR;
    }

    if (len == 0 || len > UART_TX_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }

    if (manage->huart->hdmatx == NULL)
    {
        return HAL_ERROR;
    }

    // 发送接口只允许在任务上下文调用
    if (__get_IPSR() != 0)
    {
        return HAL_ERROR;
    }

    primask = USART_Enter_Critical();
    if (manage->Tx_Busy != 0)
    {
        USART_Exit_Critical(primask);
        return HAL_BUSY;
    }
    manage->Tx_Busy = 1;
    USART_Exit_Critical(primask);

    memcpy(manage->Tx_Buffer, data, len);

    ret = HAL_UART_Transmit_DMA(manage->huart, manage->Tx_Buffer, len);
    if (ret != HAL_OK)
    {
        primask = USART_Enter_Critical();
        manage->Tx_Busy = 0;
        USART_Exit_Critical(primask);
        return ret;
    }

    return HAL_OK;
}

/*
 * @brief  UART 接收完成回调函数（由 HAL 库调用）
 * @param  huart: UART 句柄指针
 * @param  Size: 本次接收的数据长度
 * @retval 无
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    struct Struct_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);
    uint8_t *ready_buffer;

    if (manage == NULL)
    {
        return;
    }

    ready_buffer = manage->Rx_Buffer_Active;
    if (manage->Rx_Buffer_Active == manage->Rx_Buffer_0)
    {
        manage->Rx_Buffer_Active = manage->Rx_Buffer_1;
    }
    else
    {
        manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
    }

    manage->Rx_Buffer_Ready = ready_buffer;

    (void)USART_Start_Rx(manage);

    if (Size > 0 && manage->Callback_Function != NULL)
    {
        // 回调运行在中断上下文，上层如需队列请使用xQueueSendFromISR
        manage->Callback_Function(manage->Rx_Buffer_Ready, Size);
    }
}

/*
 * @brief  UART 发送完成回调函数（由 HAL 库调用）
 * @param  huart: UART 句柄指针
 * @retval 无
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    struct Struct_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);

    if (manage == NULL)
    {
        return;
    }

    manage->Tx_Busy = 0;
}

/*
 * @brief  UART 错误回调函数（由 HAL 库调用）
 * @param  huart: UART 句柄指针
 * @retval 无
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_Reinit(huart);
}
