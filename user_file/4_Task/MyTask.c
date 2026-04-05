#include "MyTask.h"

static uint8_t g_task_started = 0;
/**
 * @brief   任务初始化函数
 * @param  pramas: 无
 * @retval 无
 */
void Task_Init(void)
{
    // 设备/模块初始化统一放在任务层入口
    Gimbal_Init(NULL);
}
/**
 * @brief   任务循环函数
 * @param  pramas: 无
 * @retval 无
 */
void Task_loop(void)
{
    TaskHandle_t task_gimbal_motor_control_handle;
    TaskHandle_t task_gimbal_euler_handle;
    TaskHandle_t task_gimbal_task_handle;
    TaskHandle_t task_gimbal_manifold_control_handle;

    if (g_task_started != 0)
    {
        return;
    }

    g_task_started = 1;

    xTaskCreate(Gimbal_Motor_Control_ALL_Test, "Task_Gimbal_Motor_Control_Test", 3000, NULL, osPriorityHigh, &task_gimbal_motor_control_handle);
    xTaskCreate(Gimbal_Euler, "Task_Gimbal_Euler", 1000, NULL, osPriorityHigh, &task_gimbal_euler_handle);
    xTaskCreate(Gimbal_Task, "Task_Gimbal_Task", 2000, NULL, osPriorityNormal, &task_gimbal_task_handle);
    xTaskCreate(Gimbal_Manifold_Control, "Task_Gimbal_Manifold_Control", 2000, NULL, osPriorityNormal, &task_gimbal_manifold_control_handle);
}