#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "dvc_vofa.h"
#include "drv_usb.h"
#include "dvc_motor.h"
#include "dvc_manifold.h"
#include "dvc_bmi088.h"
#include "FreeRTOS.h"
#include "alg_quaternion.h"
#include "main.h"
#include "dvc_motor.h"
#include "spi.h"
#include "can.h"
#include <stdint.h>
#include <string.h>
void Gimbal_Init(void* pramas);
void Gimbal_Euler(void *pramas);
void Gimbal_Motor_Control_test(void* pramas);
void Gimbal_Motor_Control_ALL_Test(void* pramas);
void Gimbal_Task(void* pramas);
void Gimbal_Manifold_Control(void *pramas);
/**
 * @brief IMU 数据就绪外部中断回调
 * @note  非官方
 */
void Gimbal_IMU_EXTI_Callback(uint16_t GPIO_Pin);
#endif /* __GIMBAL_H__ */

