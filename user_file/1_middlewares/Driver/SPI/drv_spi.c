#include "drv_spi.h"
#include <stdbool.h>

#define SPI_DONE_TIMEOUT_TICKS pdMS_TO_TICKS(50)

static SemaphoreHandle_t SPI_Done_Sem = NULL;
static bool spi_dma_inited = false;

static uint8_t SPI_Tx_Buffer[16];
static uint8_t SPI_Rx_Buffer[16];

typedef struct
{
    uint8_t device;
    uint8_t *user_rx_buf;
    uint16_t valid_size;
    uint8_t device_type;
} SPI_Current_Transaction_t;

static SPI_Current_Transaction_t g_current_txn = {0};
/**
 * @brief   判断是否在中断服务程序中
 * @param  pramas: 无
 * @retval true: 在中断服务程序中
 * @retval false: 不在中断服务程序中
 */
static inline bool SPI_IsInISR(void)
{
    return (__get_IPSR() != 0);
}
/**
 * @brief   使能SPI设备的CS引脚
 * @param  device: 设备类型
 * @retval 无
 */
static void SPI_CS_High(uint8_t device)
{
    switch (device)
    {
    case ACCEL:
        ACCEL_CS_HIGH();
        break;
    case GYRO:
        GYRO_CS_HIGH();
        break;
    default:
        break;
    }
}
/**
 * @brief   失能SPI设备的CS引脚
 * @param  device: 设备类型
 * @retval 无
 */
static void SPI_CS_Low(uint8_t device)
{
    switch (device)
    {
    case ACCEL:
        ACCEL_CS_LOW();
        break;
    case GYRO:
        GYRO_CS_LOW();
        break;
    default:
        break;
    }
}
/**
 * @brief   清除SPI完成信号量
 * @param  pramas: 无
 * @retval 无
 */
static void SPI_ClearDoneSemaphore(void)
{
    if (SPI_Done_Sem == NULL)
    {
        return;
    }

    while (xSemaphoreTake(SPI_Done_Sem, 0) == pdTRUE)
    {
    }
}
/**
 * @brief   初始化SPI DMA
 * @param  pramas: 无
 * @retval 无
 */
void SPI_DMA_Init(void)
{
    if (spi_dma_inited)
    {
        return;
    }

    SPI_Done_Sem = xSemaphoreCreateCounting(1, 0);
    if (SPI_Done_Sem == NULL)
    {
        spi_dma_inited = false;
        return;
    }

    spi_dma_inited = true;
}
/**
 * @brief   从SPI设备读取寄存器数据
 * @param  hspi: SPI句柄
 * @param  device: 设备类型
 * @param  reg: 寄存器地址
 * @param  rx_data: 接收数据缓冲区
 * @param  valid_size: 有效数据大小
 * @retval HAL_StatusTypeDef: 操作状态
 */
HAL_StatusTypeDef SPI_ReadReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t *rx_data, uint16_t valid_size)
{
    if (SPI_IsInISR())
    {
        return HAL_ERROR;
    }

    if ((!spi_dma_inited) || (hspi == NULL) || (rx_data == NULL) || (valid_size == 0))
    {
        return HAL_ERROR;
    }

    uint16_t total_len = (device == ACCEL) ? (uint16_t)(valid_size + 2) : (uint16_t)(valid_size + 1);
    if (total_len > sizeof(SPI_Tx_Buffer))
    {
        return HAL_ERROR;
    }

    SPI_Tx_Buffer[0] = (uint8_t)(reg | 0x80);
    for (uint16_t i = 1; i < total_len; i++)
    {
        SPI_Tx_Buffer[i] = 0;
    }

    g_current_txn.device = device;
    g_current_txn.device_type = device;
    g_current_txn.user_rx_buf = rx_data;
    g_current_txn.valid_size = valid_size;

    SPI_ClearDoneSemaphore();
    SPI_CS_Low(device);

    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_DMA(hspi, SPI_Tx_Buffer, SPI_Rx_Buffer, total_len);
    if (ret != HAL_OK)
    {
        SPI_CS_High(device);
        return ret;
    }

    if (xSemaphoreTake(SPI_Done_Sem, SPI_DONE_TIMEOUT_TICKS) != pdTRUE)
    {
        SPI_CS_High(g_current_txn.device);
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}
/**
 * @brief   向SPI设备写入寄存器数据
 * @param  hspi: SPI句柄
 * @param  device: 设备类型
 * @param  reg: 寄存器地址
 * @param  data: 要写入的数据
 * @retval HAL_StatusTypeDef: 操作状态
 */
HAL_StatusTypeDef SPI_WriteReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t data)
{
    static uint8_t tx_buf[2];

    if (SPI_IsInISR())
    {
        return HAL_ERROR;
    }

    if ((!spi_dma_inited) || (hspi == NULL))
    {
        return HAL_ERROR;
    }

    tx_buf[0] = (uint8_t)(reg & 0x7F);
    tx_buf[1] = data;

    g_current_txn.device = device;
    g_current_txn.device_type = device;
    g_current_txn.user_rx_buf = NULL;
    g_current_txn.valid_size = 0;

    SPI_ClearDoneSemaphore();
    SPI_CS_Low(device);

    HAL_StatusTypeDef ret = HAL_SPI_Transmit_DMA(hspi, tx_buf, 2);
    if (ret != HAL_OK)
    {
        SPI_CS_High(device);
        return ret;
    }

    if (xSemaphoreTake(SPI_Done_Sem, SPI_DONE_TIMEOUT_TICKS) != pdTRUE)
    {
        SPI_CS_High(g_current_txn.device);
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}
/**
 * @brief   SPI DMA传输完成回调函数
 * @param  hspi: SPI句柄
 * @retval 无
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi == NULL) || (hspi->Instance != SPI1))
    {
        return;
    }

    SPI_CS_High(g_current_txn.device);

    if ((g_current_txn.user_rx_buf != NULL) && (g_current_txn.valid_size > 0))
    {
        uint16_t offset = (g_current_txn.device_type == ACCEL) ? 2 : 1;
        for (uint16_t i = 0; i < g_current_txn.valid_size; i++)
        {
            g_current_txn.user_rx_buf[i] = SPI_Rx_Buffer[i + offset];
        }
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(SPI_Done_Sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/**
 * @brief   SPI DMA传输完成回调函数
 * @param  hspi: SPI句柄
 * @retval 无
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi == NULL) || (hspi->Instance != SPI1))
    {
        return;
    }

    SPI_CS_High(g_current_txn.device);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(SPI_Done_Sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/**
 * @brief   SPI DMA传输错误回调函数
 * @param  hspi: SPI句柄
 * @retval 无
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi == NULL) || (hspi->Instance != SPI1))
    {
        return;
    }

    SPI_CS_High(g_current_txn.device);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(SPI_Done_Sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
