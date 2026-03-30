#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#include "main.h"
#include "stm32f4xx_hal_can.h"
#include <string.h>

// FreeRTOS相关头文件
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define CAN_RX_BUFFER_SIZE 32

typedef struct
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
} CAN_RX_MESSAGE;

// 参考 USB/USART 驱动，使用 Active/Ready 双缓冲接收 CAN 帧
struct Struct_CAN_Manage_Object
{
    CAN_HandleTypeDef *hcan;

    CAN_RX_MESSAGE Rx_Buffer_0[CAN_RX_BUFFER_SIZE];
    CAN_RX_MESSAGE Rx_Buffer_1[CAN_RX_BUFFER_SIZE];

    // 正在由中断写入的缓冲区
    CAN_RX_MESSAGE *Rx_Buffer_Active;
    uint16_t Rx_Length_Active;

    // 可被任务读取的缓冲区
    CAN_RX_MESSAGE *Rx_Buffer_Ready;
    uint16_t Rx_Length_Ready;

    // 统计信息：缓冲区满导致的丢帧数
    uint32_t Drop_Count;

    // 可选：Ready缓冲就绪时通知的任务句柄
    TaskHandle_t Notify_Task_Handle;

    // ========= 通信存活检测（Alive Guard） =========
    // 在接收回调中每收到一帧就自增，用于判断“最近100ms是否有CAN数据”
    volatile uint32_t Alive_Flag;
    // 100ms周期检查时保存上一次Alive_Flag，和当前值比较判定断联
    uint32_t Alive_Pre_Flag;
    // 当前在线状态：0=离线，1=在线
    volatile uint8_t Alive_Online;
    // 状态变化标志：0=未变化，1=状态发生变化（供任务层检查）
    volatile uint8_t Alive_Changed;
    
};

extern struct Struct_CAN_Manage_Object CAN1_Manage_Object;
extern struct Struct_CAN_Manage_Object CAN2_Manage_Object;

void CAN_Filter_Config(CAN_HandleTypeDef *hcan);
void CAN_Start(CAN_HandleTypeDef *hcan);
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t Send_id, uint8_t *data);
/**
 * @brief CAN 接收回调函数
 * @note  非官方
 */
void CAN_Receive_Callback(CAN_HandleTypeDef *hcan);

// 注册接收就绪通知任务（可选，不注册也可周期轮询读取）
void CAN_Register_Notify_Task(CAN_HandleTypeDef *hcan, TaskHandle_t task_handle);

// 读取一帧（先进先出），无数据时返回 HAL_ERROR
HAL_StatusTypeDef CAN_ReadMessage(CAN_HandleTypeDef *hcan, CAN_RX_MESSAGE *rx_message);

// 按标准ID读取一帧（读取后会从Ready缓冲移除该帧），无匹配时返回 HAL_ERROR
HAL_StatusTypeDef CAN_ReadMessage_By_StdId(CAN_HandleTypeDef *hcan, uint32_t std_id, CAN_RX_MESSAGE *rx_message);

// 100ms周期检查一次CAN链路是否在线（Flag是否增长）
void CAN_Alive_Check_100ms(CAN_HandleTypeDef *hcan);
// 获取当前在线状态：0=离线，1=在线
uint8_t CAN_Alive_IsOnline(CAN_HandleTypeDef *hcan);
// 任务层消费“在线状态变化事件”，有变化返回1并输出当前online值
uint8_t CAN_Alive_TryConsumeChanged(CAN_HandleTypeDef *hcan, uint8_t *online);

#endif /* __DRV_CAN_H__ */
