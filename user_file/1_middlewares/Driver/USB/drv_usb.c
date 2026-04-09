#include "drv_usb.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

struct Struct_USB_Manage_Object USB_Manage_Object = {0};

// USB发送内部缓存，避免上层传入栈内存后被异步发送访问到无效地址
static uint8_t USB_Tx_Buffer[USB_DATA_Send_MAX];

/**
 * @brief   获取CDC类句柄（仅在USB已枚举且类已就绪时返回非NULL）
 * @param   无
 * @retval  CDC句柄指针，未就绪返回NULL
 */
static USBD_CDC_HandleTypeDef *USB_Get_CDC_Handle(void)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return NULL;
    }

    return (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
}

/**
 * @brief   进入临界区（兼容调度器未启动阶段）
 * @param   无
 * @retval  进入前PRIMASK
 */
static uint32_t USB_Enter_Critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief   退出临界区
 * @param   primask: 进入前PRIMASK
 * @retval  无
 */
static void USB_Exit_Critical(uint32_t primask)
{
    if (primask == 0)
    {
        __enable_irq();
    }
}

/**
 * @brief   USB-RX通信存活喂狗
 * @param   无
 * @retval  无
 */
static void USB_Alive_Rx_Feed(void)
{
    USB_Manage_Object.Alive_Flag++;
}

/**
 * @brief   USB-TX通信存活喂狗
 * @param   无
 * @retval  无
 */
static void USB_Alive_Tx_Feed(void)
{
    USB_Manage_Object.Tx_Alive_Flag++;
}

/**
 * @brief   启动下一包USB接收
 * @param   无
 * @retval  无
 * @note    仅在CDC已就绪时重装接收缓冲区
 */
static void USB_Start_Receive(void)
{
    if (USB_Get_CDC_Handle() == NULL)
    {
        return;
    }

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USB_Manage_Object.Rx_Buffer_Active);
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

/**
 * @brief   双缓冲切换：将Active发布为Ready，并切换下一次接收缓冲
 * @param   无
 * @retval  无
 */
static void USB_Swap_Rx_Buffer(void)
{
    uint8_t *ready_buffer = USB_Manage_Object.Rx_Buffer_Active;

    USB_Manage_Object.Rx_Buffer_Active =
        (ready_buffer == UserRxBufferFS) ? USB_Manage_Object.Rx_Buffer_0 : UserRxBufferFS;
    USB_Manage_Object.Rx_Buffer_Ready = ready_buffer;
}

/**
 * @brief   尝试发送（不做数据拷贝）
 * @param   data: 发送数据缓冲区
 * @param   len: 发送长度
 * @retval  USBD_OK / USBD_BUSY / USBD_FAIL
 * @note    为保持与旧行为一致：CDC未就绪时返回USBD_BUSY
 */
static uint8_t USB_Try_Transmit_NoCopy(uint8_t *data, uint16_t len)
{
    USBD_CDC_HandleTypeDef *hcdc = USB_Get_CDC_Handle();

    if (hcdc == NULL)
    {
        return USBD_BUSY;
    }

    if (hcdc->TxState != 0)
    {
        return USBD_BUSY;
    }

    return CDC_Transmit_FS(data, len);
}

/**
 * @brief   设置USB最小发送间隔（tick）
 * @param   interval_tick: 最小发送间隔，传0会按1tick处理
 * @retval  无
 */
void USB_Set_Tx_Min_Interval(uint16_t interval_tick)
{
    USB_Manage_Object.Tx_Min_Interval_Tick = (interval_tick == 0) ? 1 : interval_tick;
}

/**
 * @brief   USB初始化
 * @param   callback: 接收回调函数
 * @retval  无
 */
void USB_Init(USB_Callback callback)
{
    uint32_t now_tick = HAL_GetTick();

    USB_Manage_Object.Callback_Function = callback;
    USB_Manage_Object.Rx_Buffer_Active = UserRxBufferFS;
    USB_Manage_Object.Rx_Buffer_Ready = NULL;

    USB_Manage_Object.Tx_Busy = 0;
    USB_Manage_Object.Tx_Min_Interval_Tick = USB_TX_MIN_INTERVAL_TICK_DEFAULT;
    USB_Manage_Object.Tx_Last_Transmit_Tick = now_tick - USB_TX_MIN_INTERVAL_TICK_DEFAULT;
    USB_Manage_Object.Tx_Busy_Start_Tick = now_tick;

    USB_Manage_Object.Alive_Flag = 0;
    USB_Manage_Object.Alive_Pre_Flag = 0;
    USB_Manage_Object.Tx_Alive_Flag = 0;
    USB_Manage_Object.Tx_Alive_Pre_Flag = 0;
    USB_Manage_Object.Alive_Rx_Online = 0;
    USB_Manage_Object.Alive_Tx_Online = 0;
    USB_Manage_Object.Alive_Online = 0;
    USB_Manage_Object.Alive_Changed = 0;

    USB_Start_Receive();
}

/*
 * @brief  USB发送数据函数
 * @param  data: 待发送数据指针
 * @param  len: 待发送长度
 * @retval USBD_OK / USBD_BUSY / USBD_FAIL
 */
uint8_t USB_SendData(const uint8_t *data, uint16_t len)
{
    uint8_t ret;
    uint32_t primask;
    uint32_t now_tick;
    USBD_CDC_HandleTypeDef *hcdc;

    if ((data == NULL) || (len == 0) || (len > USB_DATA_Send_MAX))
    {
        return USBD_FAIL;
    }

    // 发送接口只允许在任务上下文调用，避免中断里做大量拷贝
    if (__get_IPSR() != 0)
    {
        return USBD_FAIL;
    }

    now_tick = HAL_GetTick();
    if ((uint32_t)(now_tick - USB_Manage_Object.Tx_Last_Transmit_Tick) < USB_Manage_Object.Tx_Min_Interval_Tick)
    {
        return USBD_BUSY;
    }

    primask = USB_Enter_Critical();

    // 忙状态超时保护：防止异常情况下Tx_Busy长期不释放
    if ((USB_Manage_Object.Tx_Busy != 0) &&
        ((uint32_t)(now_tick - USB_Manage_Object.Tx_Busy_Start_Tick) >= USB_TX_BUSY_TIMEOUT_TICK))
    {
        hcdc = USB_Get_CDC_Handle();
        // 仅在“CDC未就绪”或“底层确认为不忙”时，才执行超时解锁
        if ((hcdc == NULL) || (hcdc->TxState == 0))
        {
            USB_Manage_Object.Tx_Busy = 0;
        }
    }

    if (USB_Manage_Object.Tx_Busy != 0)
    {
        USB_Exit_Critical(primask);
        return USBD_BUSY;
    }

    USB_Manage_Object.Tx_Busy = 1;
    USB_Manage_Object.Tx_Busy_Start_Tick = now_tick;
    USB_Exit_Critical(primask);

    // 先抢占发送权，再拷贝数据，避免覆盖仍在发送的缓冲区
    memcpy(USB_Tx_Buffer, data, len);

    ret = USB_Try_Transmit_NoCopy(USB_Tx_Buffer, len);
    if (ret == USBD_OK)
    {
        USB_Manage_Object.Tx_Last_Transmit_Tick = now_tick;
        return USBD_OK;
    }

    primask = USB_Enter_Critical();
    USB_Manage_Object.Tx_Busy = 0;
    USB_Exit_Critical(primask);

    return ret;
}

/*
 * @brief  USB发送字符串函数
 * @param  str: 待发送字符串
 * @retval USBD_OK / USBD_BUSY / USBD_FAIL
 */
uint8_t USB_SendString(const char *str)
{
    uint16_t len;

    if (str == NULL)
    {
        return USBD_FAIL;
    }

    len = (uint16_t)strlen(str);
    if (len == 0)
    {
        return USBD_OK;
    }

    return USB_SendData((const uint8_t *)str, len);
}

/**
 * @brief  USB接收回调函数
 * @param  buf: 接收数据缓冲区指针
 * @param  len: 接收数据长度
 * @note   非官方
 * @retval 无
 */
void USB_Rx_Callback(uint8_t *buf, uint32_t len)
{
    (void)buf;

    if (len > 0)
    {
        USB_Swap_Rx_Buffer();
        USB_Alive_Rx_Feed();

        // 先重装下一包接收，缩短NAK窗口，避免上层处理耗时影响USB时序
        USB_Start_Receive();

        if (USB_Manage_Object.Callback_Function != NULL)
        {
            USB_Manage_Object.Callback_Function(USB_Manage_Object.Rx_Buffer_Ready, len);
        }
        return;
    }

    USB_Start_Receive();
}

/**
 * @brief   USB发送完成回调函数
 * @param   无
 * @note    非官方
 * @retval  无
 */
void USB_TxCplt_Callback(void)
{
    USB_Manage_Object.Tx_Busy = 0;
    USB_Alive_Tx_Feed();
}

/**
 * @brief   100ms周期检查USB链路是否在线（RX/TX分向统计）
 * @param   无
 * @retval  无
 */
void USB_Alive_Check_100ms(void)
{
    uint8_t rx_online_new;
    uint8_t tx_online_new;
    uint8_t online_new;

    rx_online_new = (uint8_t)(USB_Manage_Object.Alive_Flag != USB_Manage_Object.Alive_Pre_Flag);
    USB_Manage_Object.Alive_Pre_Flag = USB_Manage_Object.Alive_Flag;
    tx_online_new = (uint8_t)(USB_Manage_Object.Tx_Alive_Flag != USB_Manage_Object.Tx_Alive_Pre_Flag);
    USB_Manage_Object.Tx_Alive_Pre_Flag = USB_Manage_Object.Tx_Alive_Flag;

    USB_Manage_Object.Alive_Rx_Online = rx_online_new;
    USB_Manage_Object.Alive_Tx_Online = tx_online_new;
    // 链路在线判定：RX在线 或 TX在线
    online_new = (uint8_t)((rx_online_new != 0u) || (tx_online_new != 0u));

    if (online_new != USB_Manage_Object.Alive_Online)
    {
        USB_Manage_Object.Alive_Online = online_new;
        USB_Manage_Object.Alive_Changed = 1;
    }
}

/**
 * @brief   获取当前USB链路在线状态
 * @param   无
 * @retval  0=离线 1=在线
 */
uint8_t USB_Alive_IsOnline(void)
{
    return USB_Manage_Object.Alive_Online;
}

/**
 * @brief   获取当前USB-RX在线状态
 * @param   无
 * @retval  0=离线 1=在线
 */
uint8_t USB_Alive_IsRxOnline(void)
{
    return USB_Manage_Object.Alive_Rx_Online;
}

/**
 * @brief   获取当前USB-TX在线状态
 * @param   无
 * @retval  0=离线 1=在线
 */
uint8_t USB_Alive_IsTxOnline(void)
{
    return USB_Manage_Object.Alive_Tx_Online;
}

/**
 * @brief   任务层消费一次“USB在线状态变化事件”
 * @param   online: 输出当前在线状态（可为NULL）
 * @retval  0=无变化 1=有变化
 */
uint8_t USB_Alive_TryConsumeChanged(uint8_t *online)
{
    uint8_t changed;

    changed = USB_Manage_Object.Alive_Changed;
    if (changed != 0)
    {
        USB_Manage_Object.Alive_Changed = 0;
        if (online != NULL)
        {
            *online = USB_Manage_Object.Alive_Online;
        }
    }

    return changed;
}
