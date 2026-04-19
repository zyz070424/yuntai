#include "dvc_manifold.h"
#include "stm32f405xx.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

Manifold_UART_Rx_Data Rx_Data;
volatile uint8_t Manifold_USB_Tx_Debug_Frame[MANIFOLD_USB_TX_FRAME_LEN];
volatile uint16_t Manifold_USB_Tx_Debug_Len = 0u;
volatile uint8_t Manifold_USB_Rx_Debug_Raw[MANIFOLD_USB_RX_DEBUG_RAW_MAX];
volatile uint16_t Manifold_USB_Rx_Debug_Raw_Len = 0u;
volatile uint8_t Manifold_USB_Rx_Debug_Frame[MANIFOLD_USB_RX_FRAME_LEN];
volatile uint16_t Manifold_USB_Rx_Debug_Frame_Len = 0u;
volatile uint32_t Manifold_USB_Rx_Frame_Seq = 0u;

#define MANIFOLD_RX_PITCH_ABS_MAX_DEG 90.0f
#define MANIFOLD_RX_YAW_ABS_MAX_DEG   360.0f

// 流式接收状态：仅按V2协议收包，支持“连发多包拼接”和“单包拆分跨回调”
static uint8_t manifold_rx_frame[MANIFOLD_USB_RX_FRAME_LEN];
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
 * @brief   校验视觉目标角度是否处于允许范围内。
 * @param   pitch: 俯仰角
 * @param   yaw: 偏航角
 * @retval  1=有效 0=无效
 * @note    该函数只校验数值合法性，不处理 Target_Valid 语义。
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
 * @brief   解析视觉发给电控的接收帧。
 * @param   frame: 帧起始地址（含帧头）
 * @param   pitch: 输出俯仰角
 * @param   yaw: 输出偏航角
 * @param   target_valid: 输出目标有效标志
 * @retval  1=解析成功 0=解析失败
 * @note    接收协议固定为 [Header][Yaw][Pitch][Target_Valid][Tail]。
 */
static uint8_t Manifold_Decode_Rx_Frame(const uint8_t *frame,
                                        float *pitch,
                                        float *yaw,
                                        uint8_t *target_valid)
{
    float pitch_tmp;
    float yaw_tmp;
    uint8_t target_valid_tmp;

    if ((frame == NULL) || (pitch == NULL) || (yaw == NULL) || (target_valid == NULL))
    {
        return 0u;
    }

    if (frame[MANIFOLD_USB_RX_FRAME_LEN - 1u] != Rx_Data.Frame_Tail)
    {
        return 0u;
    }

    memcpy(&yaw_tmp, frame + 1u, sizeof(float));
    memcpy(&pitch_tmp, frame + 1u + sizeof(float), sizeof(float));
    target_valid_tmp = frame[1u + sizeof(float) + sizeof(float)];

    if (Manifold_Angle_IsValid(pitch_tmp, yaw_tmp) == 0u)
    {
        return 0u;
    }

    *pitch = pitch_tmp;
    *yaw = yaw_tmp;
    *target_valid = (target_valid_tmp != 0u) ? 1u : 0u;
    return 1u;
}
/**
 * @brief   manifold USB 接收回调函数。
 * @param   Buf: 接收数据缓冲区指针
 * @param   Len: 接收数据长度
 * @retval  无
 * @note    每收到一帧新协议数据都会刷新帧序号；是否可跟踪由 Target_Valid 和上层超时逻辑共同决定。
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
    // 保存原始接收数据供调试
    memcpy((void *)Manifold_USB_Rx_Debug_Raw, Buf, raw_copy_len);
    Manifold_USB_Rx_Debug_Raw_Len = (uint16_t)raw_copy_len;

    for (i = 0; i < Len; i++)
    {
        uint8_t byte = Buf[i];
        float pitch_tmp;
        float yaw_tmp;
        uint8_t target_valid_tmp;

        // 等待帧头
        if (manifold_rx_index == 0)
        {
            if (byte == Rx_Data.Frame_Header)
            {
                manifold_rx_frame[manifold_rx_index++] = byte;
            }
            continue;
        }

        manifold_rx_frame[manifold_rx_index++] = byte;

        if (manifold_rx_index == MANIFOLD_USB_RX_FRAME_LEN)
        {
            if (Manifold_Decode_Rx_Frame(manifold_rx_frame,
                                         &pitch_tmp,
                                         &yaw_tmp,
                                         &target_valid_tmp) != 0)
            {
                memcpy((void *)Manifold_USB_Rx_Debug_Frame, manifold_rx_frame, MANIFOLD_USB_RX_FRAME_LEN);
                Manifold_USB_Rx_Debug_Frame_Len = MANIFOLD_USB_RX_FRAME_LEN;
                Rx_Data.Taget_Pitch = pitch_tmp;
                Rx_Data.Taget_Yaw = yaw_tmp;
                Rx_Data.Target_Valid = target_valid_tmp;
                Manifold_USB_Rx_Frame_Seq++;
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
    Rx_Data.Taget_Yaw = 0.0f;
    Rx_Data.Taget_Pitch = 0.0f;
    Rx_Data.Target_Valid = 0u;

    manifold_rx_index = 0u;

    USB_Init(Manifold_USB_Rx_Callback);
    // USB FS帧周期约1ms，这里留2ms发送间隔裕量，降低Busy抖动风险
    USB_Set_Tx_Min_Interval(2);
}

/**
 * @brief   清空当前视觉目标缓存。
 * @retval  无
 * @note    USB 掉线或上层复位时调用，确保角度缓存和 Target_Valid 同时失效。
 */
void Manifold_Clear_Target(void)
{
    Rx_Data.Taget_Yaw = 0.0f;
    Rx_Data.Taget_Pitch = 0.0f;
    Rx_Data.Target_Valid = 0u;
}

/**
 * @brief   manifold USB 发送数据函数
 * @param  data: manifold 数据结构体
 * @retval 无
 */
void Manifold_USB_SendData(Manifold_UART_Tx_Data *data, euler_t Euler)
{
    uint8_t tx_buf[MANIFOLD_USB_TX_FRAME_LEN];

    data->Pitch = Euler.pitch;
    data->Yaw = Euler.yaw;
    tx_buf[0] = data->Frame_Header;
    // 发送协议固定为 [Header][Yaw][Pitch][Tail]
    memcpy(tx_buf + 1, &data->Yaw, sizeof(float));
    memcpy(tx_buf + 1 + sizeof(float), &data->Pitch, sizeof(float));
    tx_buf[sizeof(tx_buf) - 1] = data->Frame_Tail;

    memcpy((void *)Manifold_USB_Tx_Debug_Frame, tx_buf, sizeof(tx_buf));
    Manifold_USB_Tx_Debug_Len = (uint16_t)sizeof(tx_buf);

    (void)USB_SendData(tx_buf, (uint16_t)sizeof(tx_buf));
}
