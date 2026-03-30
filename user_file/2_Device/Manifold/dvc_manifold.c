#include "dvc_manifold.h"
#include <string.h>

Manifold_UART_Rx_Data Rx_Data;

/**
 * @brief   manifold USB 接收回调函数
 * @param  Buf: 接收数据缓冲区指针
 * @param  Len: 接收数据长度
 * @note   非官方
 * @retval 无
 */
void Manifold_USB_Rx_Callback(uint8_t *Buf, uint32_t Len)
{
    if ((Buf == NULL) || (Len < 2))
    {
        return;
    }

    // 第一阶段：仅做帧头帧尾基础校验
    if ((Buf[0] == Rx_Data.Frame_Header) && (Buf[Len - 1] == Rx_Data.Frame_Tail))
    {
        // 第二阶段再补充完整协议解析
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
    data->Frame_Header = Frame_Header;
    data->Frame_Tail = Frame_End;
    data->Sentry_Mode = Sentry_Mode;

    Rx_Data.Frame_Header = Frame_Header;
    Rx_Data.Frame_Tail = Frame_End;

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
    uint8_t tx_buf[sizeof(euler_t) + 2];

    data->Euler_Angle.pitch = Euler.pitch;
    data->Euler_Angle.roll  = Euler.roll;
    data->Euler_Angle.yaw   = Euler.yaw;

    tx_buf[0] = data->Frame_Header;
    memcpy(tx_buf + 1, &data->Euler_Angle, sizeof(data->Euler_Angle));
    tx_buf[sizeof(data->Euler_Angle) + 1] = data->Frame_Tail;

    (void)USB_SendData(tx_buf, (uint16_t)sizeof(tx_buf));
}
