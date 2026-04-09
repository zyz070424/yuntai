#include "dvc_manifold.h"
#include "stm32f405xx.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

Manifold_UART_Rx_Data Rx_Data;
volatile uint8_t Manifold_USB_Tx_Debug_Frame[MANIFOLD_USB_FRAME_LEN_V2];
volatile uint16_t Manifold_USB_Tx_Debug_Len = 0u;
volatile uint8_t Manifold_USB_Rx_Debug_Raw[MANIFOLD_USB_RX_DEBUG_RAW_MAX];
volatile uint16_t Manifold_USB_Rx_Debug_Raw_Len = 0u;
volatile uint8_t Manifold_USB_Rx_Debug_Frame[MANIFOLD_USB_FRAME_LEN_V2];
volatile uint16_t Manifold_USB_Rx_Debug_Frame_Len = 0u;

#define MANIFOLD_RX_PITCH_ABS_MAX_DEG 90.0f
#define MANIFOLD_RX_YAW_ABS_MAX_DEG   360.0f

// 流式接收状态：仅按V2协议收包，支持“连发多包拼接”和“单包拆分跨回调”
static uint8_t manifold_rx_frame[MANIFOLD_USB_FRAME_LEN_V2];
static uint8_t manifold_rx_index = 0u;

/**
 * @brief   当前字节流重同步
 * @param   byte: 触发重同步时的当前字节
 * @retval  无
 */
static void Manifold_Rx_Resync(uint8_t byte)
{
    if (byte == Rx_Data.Frame_Header)
    {
        manifold_rx_frame[0] = byte;
        manifold_rx_index = 1u;
    }
    else
    {
        manifold_rx_index = 0u;
    }
}

/**
 * @brief   角度有效性校验
 * @param   pitch: 俯仰角
 * @param   yaw: 偏航角
 * @retval  1=有效 0=无效
 */
static uint8_t Manifold_Angle_IsValid(float pitch, float yaw)
{
    if ((isfinite(pitch) == 0) || (isfinite(yaw) == 0))
    {
        return 0u;
    }

    if ((fabsf(pitch) > MANIFOLD_RX_PITCH_ABS_MAX_DEG) ||
        (fabsf(yaw) > MANIFOLD_RX_YAW_ABS_MAX_DEG))
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief   解析 V2 帧（10字节）
 * @param   frame: 帧起始地址（含帧头）
 * @param   pitch: 输出俯仰角
 * @param   yaw: 输出偏航角
 * @retval  1=解析成功 0=解析失败
 */
static uint8_t Manifold_Decode_Frame_V2(const uint8_t *frame, float *pitch, float *yaw)
{
    float pitch_tmp;
    float yaw_tmp;

    if (frame[MANIFOLD_USB_FRAME_LEN_V2 - 1u] != Rx_Data.Frame_Tail)
    {
        return 0u;
    }

    memcpy(&pitch_tmp, frame + 1u, sizeof(float));
    memcpy(&yaw_tmp, frame + 1u + sizeof(float), sizeof(float));

    if (Manifold_Angle_IsValid(pitch_tmp, yaw_tmp) == 0u)
    {
        return 0u;
    }

    *pitch = pitch_tmp;
    *yaw = yaw_tmp;
    return 1u;
}
/**
 * @brief   manifold USB 接收回调函数
 * @param  Buf: 接收数据缓冲区指针
 * @param  Len: 接收数据长度
 * @note   非官方
 * @retval 无
 */
void Manifold_USB_Rx_Callback(uint8_t *Buf, uint32_t Len)
{
    uint32_t i;
    uint32_t raw_copy_len;

    if ((Buf == NULL) || (Len == 0u))
    {
        return;
    }

    raw_copy_len = Len;
    if (raw_copy_len > MANIFOLD_USB_RX_DEBUG_RAW_MAX)
    {
        raw_copy_len = MANIFOLD_USB_RX_DEBUG_RAW_MAX;
    }
    memcpy((void *)Manifold_USB_Rx_Debug_Raw, Buf, raw_copy_len);
    Manifold_USB_Rx_Debug_Raw_Len = (uint16_t)raw_copy_len;

    for (i = 0u; i < Len; i++)
    {
        uint8_t byte = Buf[i];
        float pitch_tmp;
        float yaw_tmp;

        // 等待帧头
        if (manifold_rx_index == 0u)
        {
            if (byte == Rx_Data.Frame_Header)
            {
                manifold_rx_frame[manifold_rx_index++] = byte;
            }
            continue;
        }

        manifold_rx_frame[manifold_rx_index++] = byte;

        if (manifold_rx_index == MANIFOLD_USB_FRAME_LEN_V2)
        {
            if (Manifold_Decode_Frame_V2(manifold_rx_frame, &pitch_tmp, &yaw_tmp) != 0u)
            {
                memcpy((void *)Manifold_USB_Rx_Debug_Frame, manifold_rx_frame, MANIFOLD_USB_FRAME_LEN_V2);
                Manifold_USB_Rx_Debug_Frame_Len = MANIFOLD_USB_FRAME_LEN_V2;
                Rx_Data.Taget_Pitch = pitch_tmp;
                Rx_Data.Taget_Yaw = yaw_tmp;
                manifold_rx_index = 0u;
            }
            else
            {
                Manifold_Rx_Resync(byte);
            }
        }
    }
}

/**
 * @brief   manifold 初始化函数
 * @param  data: manifold 数据结构体指针
 * @param  Frame_Header: 帧头字节
 * @param  Frame_End: 帧尾字节
 * @param  Sentry_Mode: 哨兵模式状态
 * @retval 无
 */
void Manifold_Init(Manifold_UART_Tx_Data *data, uint8_t Frame_Header, uint8_t Frame_End, enum Enum_Manifold_Sentry_Mode Sentry_Mode)
{
    (void)Sentry_Mode;
    data->Frame_Header = Frame_Header;
    data->Frame_Tail = Frame_End;

    Rx_Data.Frame_Header = Frame_Header;
    Rx_Data.Frame_Tail = Frame_End;
    Rx_Data.Taget_Pitch = 0.0f;
    Rx_Data.Taget_Yaw = 0.0f;

    manifold_rx_index = 0u;

    USB_Init(Manifold_USB_Rx_Callback);
    // USB FS帧周期约1ms，这里留2ms发送间隔裕量，降低Busy抖动风险
    USB_Set_Tx_Min_Interval(2);
}

/**
 * @brief   manifold USB 发送数据函数
 * @param  data: manifold 数据结构体
 * @retval 无
 */
void Manifold_USB_SendData(Manifold_UART_Tx_Data *data, euler_t Euler)
{
    uint8_t tx_buf[MANIFOLD_USB_FRAME_LEN_V2];

    data->Pitch = Euler.pitch;
    data->Yaw = Euler.yaw;
    tx_buf[0] = data->Frame_Header;
    // V2协议固定为 [Header][Pitch][Yaw][Tail]
    memcpy(tx_buf + 1, &data->Pitch, sizeof(float));
    memcpy(tx_buf + 1 + sizeof(float), &data->Yaw, sizeof(float));
    tx_buf[sizeof(tx_buf) - 1] = data->Frame_Tail;

    memcpy((void *)Manifold_USB_Tx_Debug_Frame, tx_buf, sizeof(tx_buf));
    Manifold_USB_Tx_Debug_Len = (uint16_t)sizeof(tx_buf);

    (void)USB_SendData(tx_buf, (uint16_t)sizeof(tx_buf));
}
