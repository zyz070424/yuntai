#ifndef __TASK_H__
#define __TASK_H__

#include "Gimbal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include <string.h>

void Task_Init(void);
void Task_loop(void);

#endif /* __TASK_H__ */