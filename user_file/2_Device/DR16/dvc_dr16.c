#include "dvc_dr16.h"
#include <string.h>

/* 全局双缓冲：ISR写入最新帧，任务读取稳定帧 */
static uint8_t g_dr16_frame_buf[2][DR16_FRAME_LEN];
static volatile uint8_t g_dr16_ready_index = 0;
static volatile uint8_t g_dr16_has_new_frame = 0;
/**
 * @brief  限幅函数，避免归一化越界
 * @param  val: 输入值
 * @param  min_val: 最小允许值
 * @param  max_val: 最大允许值
 * @return float: 限幅后的值
 * */
static float DR16_Clamp(float val, float min_val, float max_val)
{
    if (val < min_val)
    {
        return min_val;
    }
    if (val > max_val)
    {
        return max_val;
    }
    return val;
}

/**
 * @brief  开关值合法化
 * @param  sw: 输入开关值
 * @return uint8_t: 合法化后的开关值
 * */
static uint8_t DR16_SanitizeSwitch(uint8_t sw)
{
    if ((sw == DR16_SWITCH_UP) || (sw == DR16_SWITCH_MIDDLE) || (sw == DR16_SWITCH_DOWN))
    {
        return sw;
    }
    return DR16_SWITCH_MIDDLE;
}

/**
 * @brief  从双缓冲中取一帧最新数据（无队列）
 * @param  out_frame: 输出数据缓冲区指针
 * @return uint8_t: 是否成功获取数据
 * @retval 无
 * */
static uint8_t DR16_FetchLatestFrame(uint8_t *out_frame)
{
    uint32_t primask;
    uint8_t index;

    if (out_frame == NULL)
    {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (g_dr16_has_new_frame == 0)
    {
        if (primask == 0)
        {
            __enable_irq();
        }
        return 0;
    }

    index = g_dr16_ready_index;
    memcpy(out_frame, g_dr16_frame_buf[index], DR16_FRAME_LEN);
    g_dr16_has_new_frame = 0;

    if (primask == 0)
    {
        __enable_irq();
    }

    return 1;
}

/**
 * @brief  DR16 接收回调函数
 * @param  data: 接收数据缓冲区指针
 * @param  len: 接收数据长度
 * @retval 无
 */
static void DR16_Receive_Callback(uint8_t *data, uint16_t len)
{
    uint8_t write_index;

    if ((data == NULL) || (len < DR16_FRAME_LEN))
    {
        return;
    }

    /* 写到非ready缓冲区，完成后再原子z切换索引 */
    write_index = (uint8_t)(g_dr16_ready_index ^ 1);
    memcpy(g_dr16_frame_buf[write_index], data, DR16_FRAME_LEN);
    __DMB();
    g_dr16_ready_index = write_index;
    g_dr16_has_new_frame = 1;
}
/**
 * @brief  判断开关状态
 * @param  sw: 输出开关状态指针
 * @param  now: 当前开关值
 * @param  prev: 上一开关值
 * @retval 无
 * */
static void JudgeSwitch(DR16_Switch_Status_TypeDef *sw, uint8_t now, uint8_t prev)
{
    now = DR16_SanitizeSwitch(now);
    prev = DR16_SanitizeSwitch(prev);

    switch (prev)
    {
    case DR16_SWITCH_UP:
        if (now == DR16_SWITCH_UP)
        {
            *sw = DR16_SWITCH_STATUS_UP;
        }
        else if (now == DR16_SWITCH_MIDDLE)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_UP_MIDDLE;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN;
        }
        break;

    case DR16_SWITCH_DOWN:
        if (now == DR16_SWITCH_DOWN)
        {
            *sw = DR16_SWITCH_STATUS_DOWN;
        }
        else if (now == DR16_SWITCH_MIDDLE)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_DOWN_MIDDLE;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP;
        }
        break;

    case DR16_SWITCH_MIDDLE:
    default:
        if (now == DR16_SWITCH_UP)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP;
        }
        else if (now == DR16_SWITCH_DOWN)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_MIDDLE;
        }
        break;
    }
}
/** 
 * @brief  判断按键状态
 * @param  key: 输出按键状态指针
 * @param  now: 当前按键值
 * @param  prev: 上一按键值
 * @note   非官方
 * @retval 无
 * */
static void JudgeKey(DR16_Key_Status_TypeDef *key, uint8_t now, uint8_t prev)
{
    if (prev == DR16_KEY_FREE)
    {
        if (now == DR16_KEY_FREE)
        {
            *key = DR16_KEY_STATUS_FREE;
        }
        else
        {
            *key = DR16_KEY_STATUS_TRIG_FREE_PRESSED;
        }
    }
    else
    {
        if (now == DR16_KEY_FREE)
        {
            *key = DR16_KEY_STATUS_TRIG_PRESSED_FREE;
        }
        else
        {
            *key = DR16_KEY_STATUS_PRESSED;
        }
    }
}
/**
 * @brief  初始化DR16
 * @param  huart: UART句柄指针
 * @retval 无
 * */
void DR16_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    g_dr16_ready_index = 0;
    g_dr16_has_new_frame = 0;
    memset(g_dr16_frame_buf, 0, sizeof(g_dr16_frame_buf));

    USART_Init(huart, DR16_Receive_Callback);
}

/**
 * @brief 处理DR16数据（非阻塞）
 * @note 无新帧时直接返回，不阻塞任务
 */
void DR16_Process(DR16_DataTypeDef *dr16)
{
    uint8_t raw[DR16_FRAME_LEN];
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    float rocker_denom;

    if (dr16 == NULL)
    {
        return;
    }

    if (DR16_FetchLatestFrame(raw) == 0)
    {
        return;
    }

    ch0 = ((uint16_t)raw[0] | ((uint16_t)(raw[1] & 0x07) << 8)) & 0x07FF;
    ch1 = (((uint16_t)raw[1] >> 3) | ((uint16_t)(raw[2] & 0x3F) << 5)) & 0x07FF;
    ch2 = (((uint16_t)raw[2] >> 6) | ((uint16_t)raw[3] << 2) | ((uint16_t)(raw[4] & 0x01) << 10)) & 0x07FF;
    ch3 = (((uint16_t)raw[4] >> 1) | ((uint16_t)(raw[5] & 0x0F) << 7)) & 0x07FF;

    /* DJI常用定义：s1高2位，s2次高2位 */
    dr16->raw_s1 = (raw[5] >> 6) & 0x03;
    dr16->raw_s2 = (raw[5] >> 4) & 0x03;

    mouse_x = (int16_t)((uint16_t)raw[6]  | ((uint16_t)raw[7]  << 8));
    mouse_y = (int16_t)((uint16_t)raw[8]  | ((uint16_t)raw[9]  << 8));
    mouse_z = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));

    dr16->raw_mouse_l = raw[12];
    dr16->raw_mouse_r = raw[13];
    dr16->raw_key = (uint16_t)raw[14] | ((uint16_t)raw[15] << 8);

    rocker_denom = DR16_ROCKER_RANGE / 2.0f;

    dr16->right_x = DR16_Clamp((ch0 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->right_y = DR16_Clamp((ch1 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->left_x  = DR16_Clamp((ch2 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->left_y  = DR16_Clamp((ch3 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);

    dr16->mouse_x = DR16_Clamp(mouse_x / 32768.0f, -1.0f, 1.0f);
    dr16->mouse_y = DR16_Clamp(mouse_y / 32768.0f, -1.0f, 1.0f);
    dr16->mouse_z = DR16_Clamp(mouse_z / 32768.0f, -1.0f, 1.0f);
}

/**
 * @brief  DR16 1ms 周期回调函数
 * @param  dr16: DR16 数据结构体指针
 * @note   非官方
 * @retval 无
 */
void DR16_Timer1msCallback(DR16_DataTypeDef *dr16)
{
    uint8_t i;
    uint8_t now_bit;
    uint8_t prev_bit;

    if (dr16 == NULL)
    {
        return;
    }

    JudgeSwitch(&dr16->left_switch, dr16->raw_s1, dr16->prev_raw_s1);
    JudgeSwitch(&dr16->right_switch, dr16->raw_s2, dr16->prev_raw_s2);

    JudgeKey(&dr16->mouse_left, dr16->raw_mouse_l, dr16->prev_raw_mouse_l);
    JudgeKey(&dr16->mouse_right, dr16->raw_mouse_r, dr16->prev_raw_mouse_r);

    for (i = 0; i < 16; i++)
    {
        now_bit = (dr16->raw_key >> i) & 0x01;
        prev_bit = (dr16->prev_raw_key >> i) & 0x01;
        JudgeKey(&dr16->key[i], now_bit, prev_bit);
    }

    dr16->prev_raw_s1 = dr16->raw_s1;
    dr16->prev_raw_s2 = dr16->raw_s2;
    dr16->prev_raw_mouse_l = dr16->raw_mouse_l;
    dr16->prev_raw_mouse_r = dr16->raw_mouse_r;
    dr16->prev_raw_key = dr16->raw_key;
}
