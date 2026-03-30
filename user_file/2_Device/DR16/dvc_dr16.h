#ifndef __DVC_DR16_H__
#define __DVC_DR16_H__

#include <stdint.h>
#include "drv_usart.h"

#define DR16_FRAME_LEN      18

#define DR16_SWITCH_UP      1
#define DR16_SWITCH_DOWN    2
#define DR16_SWITCH_MIDDLE  3

#define DR16_KEY_FREE       0
#define DR16_KEY_PRESSED    1

#define DR16_ROCKER_OFFSET  1024
#define DR16_ROCKER_RANGE   1320   // 1684-364

#define DR16_KEY_W      0
#define DR16_KEY_S      1
#define DR16_KEY_A      2
#define DR16_KEY_D      3
#define DR16_KEY_SHIFT  4
#define DR16_KEY_CTRL   5
#define DR16_KEY_Q      6
#define DR16_KEY_E      7
#define DR16_KEY_R      8
#define DR16_KEY_F      9
#define DR16_KEY_G      10
#define DR16_KEY_Z      11
#define DR16_KEY_X      12
#define DR16_KEY_C      13
#define DR16_KEY_V      14
#define DR16_KEY_B      15

typedef enum
{
    DR16_SWITCH_STATUS_UP = 0,
    DR16_SWITCH_STATUS_MIDDLE,
    DR16_SWITCH_STATUS_DOWN,
    DR16_SWITCH_STATUS_TRIG_UP_MIDDLE,
    DR16_SWITCH_STATUS_TRIG_MIDDLE_UP,
    DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN,
    DR16_SWITCH_STATUS_TRIG_DOWN_MIDDLE,
} DR16_Switch_Status_TypeDef;

typedef enum
{
    DR16_KEY_STATUS_FREE = 0,
    DR16_KEY_STATUS_PRESSED,
    DR16_KEY_STATUS_TRIG_FREE_PRESSED,
    DR16_KEY_STATUS_TRIG_PRESSED_FREE,
} DR16_Key_Status_TypeDef;

// 解析后的遥控器数据（供应用程序使用）
typedef struct
{
    float right_x;
    float right_y;
    float left_x;
    float left_y;

    float mouse_x;
    float mouse_y;
    float mouse_z;

    DR16_Switch_Status_TypeDef left_switch;
    DR16_Switch_Status_TypeDef right_switch;

    DR16_Key_Status_TypeDef mouse_left;
    DR16_Key_Status_TypeDef mouse_right;

    DR16_Key_Status_TypeDef key[16];

    uint8_t raw_s1;
    uint8_t raw_s2;
    uint8_t raw_mouse_l;
    uint8_t raw_mouse_r;
    uint16_t raw_key;

    uint8_t prev_raw_s1;
    uint8_t prev_raw_s2;
    uint8_t prev_raw_mouse_l;
    uint8_t prev_raw_mouse_r;
    uint16_t prev_raw_key;
} DR16_DataTypeDef;

void DR16_Init(UART_HandleTypeDef *huart);
void DR16_Process(DR16_DataTypeDef *dr16);
/**
 * @brief DR16 1ms 周期回调函数
 * @note  非官方
 */
void DR16_Timer1msCallback(DR16_DataTypeDef *dr16);

#endif /* __DVC_DR16_H__ */
