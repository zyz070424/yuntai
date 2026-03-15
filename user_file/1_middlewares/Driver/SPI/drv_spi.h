
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

void SPI_DMA_Init(void);
HAL_StatusTypeDef SPI_WriteReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t data);
HAL_StatusTypeDef SPI_ReadReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t *rx_data, uint16_t valid_size);
#endif /* __DRV_SPI_H__ */
