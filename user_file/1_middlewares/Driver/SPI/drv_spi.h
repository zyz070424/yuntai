
#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "main.h"
#include "projdefs.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include <stdint.h>
enum SPI_Device
{
    ACCEL = 0,
    GYRO = 1,
    // BMI088温度寄存器属于ACCEL寄存器组，TEMP仅作为逻辑设备标识
    TEMP = 2,
};
/*
     * @brief  加速度计、陀螺仪、温度传感器CS引脚高电平
     * @param  无
     * @retval 无
     */
#define ACCEL_CS_HIGH()  HAL_GPIO_WritePin(ACCEL_CSB1_GPIO_Port, ACCEL_CSB1_Pin, GPIO_PIN_SET)
#define ACCEL_CS_LOW()   HAL_GPIO_WritePin(ACCEL_CSB1_GPIO_Port, ACCEL_CSB1_Pin, GPIO_PIN_RESET)

#define GYRO_CS_HIGH()   HAL_GPIO_WritePin(GYRO_CSB2_GPIO_Port, GYRO_CSB2_Pin, GPIO_PIN_SET)
#define GYRO_CS_LOW()    HAL_GPIO_WritePin(GYRO_CSB2_GPIO_Port, GYRO_CSB2_Pin, GPIO_PIN_RESET)

/**
 * @brief SPI当前事务上下文
 *
 */
struct Struct_SPI_Current_Transaction
{
    uint8_t device;
    uint8_t *user_rx_buf;
    uint16_t valid_size;
};

/**
 * @brief SPI管理对象（风格与CAN驱动统一）
 *
 */
struct Struct_SPI_Manage_Object
{
    SPI_HandleTypeDef *hspi;

    // DMA同步信号量
    SemaphoreHandle_t Done_Sem;
    // DMA初始化标志：0=未初始化 1=已初始化
    uint8_t DMA_Inited;

    // SPI DMA收发缓冲区
    uint8_t Tx_Buffer[16];
    uint8_t Rx_Buffer[16];

    // 当前事务上下文
    struct Struct_SPI_Current_Transaction Current_Transaction;

    // 事务状态：0=NONE 1=OK 2=ERROR 3=TIMEOUT
    volatile uint8_t Transfer_State;

    // ========= 通信存活检测（Alive Guard） =========
    // 在事务成功时自增，用于判断“最近100ms是否有有效SPI通信”
    volatile uint32_t Alive_Flag;
    // 100ms周期检查时保存上一次Alive_Flag，和当前值比较判定断联
    uint32_t Alive_Pre_Flag;
    // 当前在线状态：0=离线，1=在线
    volatile uint8_t Alive_Online;
    // 状态变化标志：0=未变化，1=状态发生变化（供任务层消费）
    volatile uint8_t Alive_Changed;
};

extern struct Struct_SPI_Manage_Object SPI1_Manage_Object;

void SPI_DMA_Init(void);
HAL_StatusTypeDef SPI_WriteReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t data);
HAL_StatusTypeDef SPI_ReadReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t *rx_data, uint16_t valid_size);

// 100ms周期检查一次SPI链路是否在线（Flag是否增长）
void SPI_Alive_Check_100ms(void);
// 获取当前在线状态：0=离线，1=在线
uint8_t SPI_Alive_IsOnline(void);
// 任务层消费“在线状态变化事件”，有变化返回1并输出当前online值
uint8_t SPI_Alive_TryConsumeChanged(uint8_t *online);

#endif /* __DRV_SPI_H__ */
