#include "drv_spi.h"

#define SPI_DONE_TIMEOUT_TICKS pdMS_TO_TICKS(50)
#define SPI_TRANSFER_STATE_NONE 0
#define SPI_TRANSFER_STATE_OK 1
#define SPI_TRANSFER_STATE_ERROR 2
#define SPI_TRANSFER_STATE_TIMEOUT 3

// SPI1管理对象实例
struct Struct_SPI_Manage_Object SPI1_Manage_Object = {0};

/**
 * @brief   获取SPI管理对象
 * @param  hspi: SPI句柄
 * @retval 管理对象指针，失败返回NULL
 */
static struct Struct_SPI_Manage_Object *SPI_Get_Manage_Object(const SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL)
    {
        return NULL;
    }

    if (hspi->Instance == SPI1)
    {
        return &SPI1_Manage_Object;
    }

    return NULL;
}

/**
 * @brief   判断设备枚举是否在有效范围内
 * @param  device: 设备枚举值
 * @retval 0=无效 1=有效
 */
static uint8_t SPI_Device_IsValid(uint8_t device)
{
    return (device <= TEMP) ? 1 : 0;
}

/**
 * @brief   获取设备读取偏移
 * @param  device: 设备枚举值
 * @retval 偏移字节数
 * @note   非法设备默认按1字节偏移处理（防御性返回）
 */
static uint8_t SPI_Device_GetReadOffset(uint8_t device)
{
    switch (device)
    {
    case ACCEL:
    case TEMP:
        return 2;
    case GYRO:
        return 1;
    default:
        return 1;
    }
}

/**
 * @brief   控制设备片选
 * @param  device: 设备枚举值
 * @param  level_high: 0=拉低 1=拉高
 * @retval 无
 */
static void SPI_Device_CS_Write(uint8_t device, uint8_t level_high)
{
    if (SPI_Device_IsValid(device) == 0)
    {
        return;
    }

    switch (device)
    {
    case ACCEL:
        if (level_high == 0)
        {
            ACCEL_CS_LOW();
        }
        else
        {
            ACCEL_CS_HIGH();
        }
        break;
    case GYRO:
        if (level_high == 0)
        {
            GYRO_CS_LOW();
        }
        else
        {
            GYRO_CS_HIGH();
        }
        break;
    case TEMP:
        // BMI088温度寄存器在ACCEL寄存器组，片选与ACCEL一致
        if (level_high == 0)
        {
            ACCEL_CS_LOW();
        }
        else
        {
            ACCEL_CS_HIGH();
        }
        break;
    default:
        break;
    }
}

/**
 * @brief   判断是否在中断服务程序中
 * @param  无
 * @retval 1=在ISR 0=不在ISR
 */
static inline uint8_t SPI_IsInISR(void)
{
    return (__get_IPSR() != 0) ? 1 : 0;
}

/**
 * @brief   清除SPI完成信号量
 * @param  manage: SPI管理对象
 * @retval 无
 */
static void SPI_ClearDoneSemaphore(struct Struct_SPI_Manage_Object *manage)
{
    if ((manage == NULL) || (manage->Done_Sem == NULL))
    {
        return;
    }

    while (xSemaphoreTake(manage->Done_Sem, 0) == pdTRUE)
    {
    }
}

/**
 * @brief   SPI通信存活喂狗
 * @param  manage: SPI管理对象
 * @retval 无
 * @note   仅在事务成功时调用
 */
static void SPI_Alive_Feed(struct Struct_SPI_Manage_Object *manage)
{
    if (manage == NULL)
    {
        return;
    }

    manage->Alive_Flag++;
}

/**
 * @brief   等待SPI DMA事务完成并返回结果
 * @param  manage: SPI管理对象
 * @param  hspi: SPI句柄
 * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
 */
static HAL_StatusTypeDef SPI_Wait_Transfer_Done(struct Struct_SPI_Manage_Object *manage, SPI_HandleTypeDef *hspi)
{
    if ((manage == NULL) || (manage->Done_Sem == NULL))
    {
        return HAL_ERROR;
    }

    if (xSemaphoreTake(manage->Done_Sem, SPI_DONE_TIMEOUT_TICKS) != pdTRUE)
    {
        manage->Transfer_State = SPI_TRANSFER_STATE_TIMEOUT;
        SPI_Device_CS_Write(manage->Current_Transaction.device, 1);
        (void)HAL_SPI_Abort(hspi);
        return HAL_TIMEOUT;
    }

    if (manage->Transfer_State == SPI_TRANSFER_STATE_OK)
    {
        SPI_Alive_Feed(manage);
        return HAL_OK;
    }

    return HAL_ERROR;
}

/**
 * @brief   初始化SPI DMA
 * @param  无
 * @retval 无
 */
void SPI_DMA_Init(void)
{
    struct Struct_SPI_Manage_Object *manage = &SPI1_Manage_Object;

    if (manage->DMA_Inited != 0)
    {
        return;
    }

    manage->Done_Sem = xSemaphoreCreateCounting(1, 0);
    if (manage->Done_Sem == NULL)
    {
        manage->DMA_Inited = 0;
        return;
    }

    manage->DMA_Inited = 1;
    manage->Transfer_State = SPI_TRANSFER_STATE_NONE;

    // 初始化SPI链路存活检测状态
    manage->Alive_Flag = 0;
    manage->Alive_Pre_Flag = 0;
    manage->Alive_Online = 0;
    manage->Alive_Changed = 0;

    // 初始化事务上下文
    manage->Current_Transaction.device = ACCEL;
    manage->Current_Transaction.user_rx_buf = NULL;
    manage->Current_Transaction.valid_size = 0;
}

/**
 * @brief   从SPI设备读取寄存器数据
 * @param  hspi: SPI句柄
 * @param  device: 设备类型
 * @param  reg: 寄存器地址
 * @param  rx_data: 接收缓冲区
 * @param  valid_size: 有效数据长度
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef SPI_ReadReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t *rx_data, uint16_t valid_size)
{
    struct Struct_SPI_Manage_Object *manage;
    uint16_t total_len;
    uint16_t i;
    HAL_StatusTypeDef ret;

    if (SPI_IsInISR() != 0)
    {
        return HAL_ERROR;
    }

    manage = SPI_Get_Manage_Object(hspi);
    if ((manage == NULL) || (manage->DMA_Inited == 0) || (SPI_Device_IsValid(device) == 0) || (rx_data == NULL) || (valid_size == 0))
    {
        return HAL_ERROR;
    }

    total_len = (uint16_t)(valid_size + SPI_Device_GetReadOffset(device));
    if (total_len > sizeof(manage->Tx_Buffer))
    {
        return HAL_ERROR;
    }

    manage->Tx_Buffer[0] = (uint8_t)(reg | 0x80);
    for (i = 1; i < total_len; i++)
    {
        manage->Tx_Buffer[i] = 0;
    }

    manage->Current_Transaction.device = device;
    manage->Current_Transaction.user_rx_buf = rx_data;
    manage->Current_Transaction.valid_size = valid_size;
    manage->Transfer_State = SPI_TRANSFER_STATE_NONE;

    SPI_ClearDoneSemaphore(manage);
    SPI_Device_CS_Write(device, 0);

    ret = HAL_SPI_TransmitReceive_DMA(hspi, manage->Tx_Buffer, manage->Rx_Buffer, total_len);
    if (ret != HAL_OK)
    {
        SPI_Device_CS_Write(device, 1);
        manage->Transfer_State = SPI_TRANSFER_STATE_ERROR;
        return ret;
    }

    return SPI_Wait_Transfer_Done(manage, hspi);
}

/**
 * @brief   向SPI设备写入寄存器数据
 * @param  hspi: SPI句柄
 * @param  device: 设备类型
 * @param  reg: 寄存器地址
 * @param  data: 写入数据
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef SPI_WriteReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t data)
{
    struct Struct_SPI_Manage_Object *manage;
    HAL_StatusTypeDef ret;

    if (SPI_IsInISR() != 0)
    {
        return HAL_ERROR;
    }

    manage = SPI_Get_Manage_Object(hspi);
    if ((manage == NULL) || (manage->DMA_Inited == 0) || (SPI_Device_IsValid(device) == 0))
    {
        return HAL_ERROR;
    }

    manage->Tx_Buffer[0] = (uint8_t)(reg & 0x7F);
    manage->Tx_Buffer[1] = data;

    manage->Current_Transaction.device = device;
    manage->Current_Transaction.user_rx_buf = NULL;
    manage->Current_Transaction.valid_size = 0;
    manage->Transfer_State = SPI_TRANSFER_STATE_NONE;

    SPI_ClearDoneSemaphore(manage);
    SPI_Device_CS_Write(device, 0);

    ret = HAL_SPI_Transmit_DMA(hspi, manage->Tx_Buffer, 2);
    if (ret != HAL_OK)
    {
        SPI_Device_CS_Write(device, 1);
        manage->Transfer_State = SPI_TRANSFER_STATE_ERROR;
        return ret;
    }

    return SPI_Wait_Transfer_Done(manage, hspi);
}

/**
 * @brief   ISR内收尾：统一处理SPI事务结束
 * @param  hspi: SPI句柄
 * @param  transfer_ok: 1=成功 0=失败
 * @param  copy_rx: 1=需要拷贝接收数据 0=不拷贝
 * @retval 无
 */
static void SPI_Transfer_Finish_FromISR(SPI_HandleTypeDef *hspi, uint8_t transfer_ok, uint8_t copy_rx)
{
    struct Struct_SPI_Manage_Object *manage = SPI_Get_Manage_Object(hspi);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t i;
    uint16_t offset;

    if ((manage == NULL) || (manage->Done_Sem == NULL))
    {
        return;
    }

    SPI_Device_CS_Write(manage->Current_Transaction.device, 1);

    if ((copy_rx != 0) && (manage->Current_Transaction.user_rx_buf != NULL) && (manage->Current_Transaction.valid_size > 0))
    {
        offset = SPI_Device_GetReadOffset(manage->Current_Transaction.device);
        if ((uint16_t)(manage->Current_Transaction.valid_size + offset) > sizeof(manage->Rx_Buffer))
        {
            transfer_ok = 0;
        }
        else
        {
            for (i = 0; i < manage->Current_Transaction.valid_size; i++)
            {
                manage->Current_Transaction.user_rx_buf[i] = manage->Rx_Buffer[i + offset];
            }
        }
    }

    manage->Transfer_State = (transfer_ok != 0) ? SPI_TRANSFER_STATE_OK : SPI_TRANSFER_STATE_ERROR;

    xSemaphoreGiveFromISR(manage->Done_Sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief   SPI DMA收发完成回调
 * @param  hspi: SPI句柄
 * @retval 无
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Transfer_Finish_FromISR(hspi, 1, 1);
}

/**
 * @brief   SPI DMA发送完成回调
 * @param  hspi: SPI句柄
 * @retval 无
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Transfer_Finish_FromISR(hspi, 1, 0);
}

/**
 * @brief   SPI DMA错误回调
 * @param  hspi: SPI句柄
 * @retval 无
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Transfer_Finish_FromISR(hspi, 0, 0);
}

/**
 * @brief   100ms周期检查SPI链路是否在线（Flag/Pre_Flag机制）
 * @param  无
 * @retval 无
 */
void SPI_Alive_Check_100ms(void)
{
    struct Struct_SPI_Manage_Object *manage = &SPI1_Manage_Object;
    uint8_t online_new;

    if (manage->DMA_Inited == 0)
    {
        return;
    }

    online_new = (uint8_t)(manage->Alive_Flag != manage->Alive_Pre_Flag);
    manage->Alive_Pre_Flag = manage->Alive_Flag;

    if (online_new != manage->Alive_Online)
    {
        manage->Alive_Online = online_new;
        manage->Alive_Changed = 1;
    }
}

/**
 * @brief   获取SPI链路在线状态
 * @param  无
 * @retval 0=离线 1=在线
 */
uint8_t SPI_Alive_IsOnline(void)
{
    return SPI1_Manage_Object.Alive_Online;
}

/**
 * @brief   消费一次SPI在线状态变化事件
 * @param  online: 输出当前在线状态（可为NULL）
 * @retval 0=无变化 1=有变化
 */
uint8_t SPI_Alive_TryConsumeChanged(uint8_t *online)
{
    struct Struct_SPI_Manage_Object *manage = &SPI1_Manage_Object;
    uint8_t changed;

    changed = manage->Alive_Changed;
    if (changed != 0)
    {
        manage->Alive_Changed = 0;
        if (online != NULL)
        {
            *online = manage->Alive_Online;
        }
    }

    return changed;
}
