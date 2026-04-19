#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "drv_usb.h"
#include "dvc_motor.h"
#include "dvc_manifold.h"
#include "dvc_bmi088.h"
#include "FreeRTOS.h"
#include "alg_quaternion.h"
#include "main.h"
#include "spi.h"
#include "can.h"
#include <stdint.h>
#include <string.h>
#include "alg_pid.h"
#include "gimbal_sentry.h"
#include "alg_dwt.h"
#include "portmacro.h"
#include "stm32f4xx_hal.h"
#include <math.h>

void Gimbal_Init(void* pramas);
void Gimbal_Euler(void *pramas);
void Gimbal_Motor_Control_ALL_Test(void* pramas);
void Gimbal_Task(void* pramas);
void Gimbal_Manifold_Control(void *pramas);
/**
 * @brief IMU 数据就绪外部中断回调
 * @note  非官方
 */
void Gimbal_IMU_EXTI_Callback(uint16_t GPIO_Pin);
#endif /* __GIMBAL_H__ */

