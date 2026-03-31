/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define ACCEL_CSB1_Pin GPIO_PIN_4
#define ACCEL_CSB1_GPIO_Port GPIOC
#define GYRO_INT_Pin GPIO_PIN_5
#define GYRO_INT_GPIO_Port GPIOC
#define GYRO_INT_EXTI_IRQn EXTI9_5_IRQn
#define ACCEL_INT_Pin GPIO_PIN_0
#define ACCEL_INT_GPIO_Port GPIOB
#define ACCEL_INT_EXTI_IRQn EXTI0_IRQn
#define GYRO_CSB2_Pin GPIO_PIN_1
#define GYRO_CSB2_GPIO_Port GPIOB
#define TEMP_088_Pin GPIO_PIN_10
#define TEMP_088_GPIO_Port GPIOB
#define DR16_Pin GPIO_PIN_11
#define DR16_GPIO_Port GPIOB
#define USART6_TX_Pin GPIO_PIN_6
#define USART6_TX_GPIO_Port GPIOC
#define USART6_RX_Pin GPIO_PIN_7
#define USART6_RX_GPIO_Port GPIOC
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USB1_DM_Pin GPIO_PIN_11
#define USB1_DM_GPIO_Port GPIOA
#define USB1_DP_Pin GPIO_PIN_12
#define USB1_DP_GPIO_Port GPIOA
#define CAN2_RX_Pin GPIO_PIN_5
#define CAN2_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_6
#define CAN2_TX_GPIO_Port GPIOB
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB
#define CAN1_RX_Pin GPIO_PIN_8
#define CAN1_RX_GPIO_Port GPIOB
#define CAN1_TX_Pin GPIO_PIN_9
#define CAN1_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
