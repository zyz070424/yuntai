#include "drv_usb.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

struct Struct_USB_Manage_Object USB_Manage_Object = {0};

// USB发送内部缓存，避免上层传入栈内存后被异步发送访问到无效地址
static uint8_t USB_Tx_Buffer[USB_DATA_Send_MAX];

// 判断USB CDC是否已配置完成
static uint8_t USB_Is_Ready(void)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return 0;
    }

    if (hUsbDeviceFS.pClassData == NULL)
    {
        return 0;
    }

    return 1;
}

// 判断USB CDC发送是否忙
static uint8_t USB_Is_Tx_Busy(void)
{
    USBD_CDC_HandleTypeDef *hcdc;

    if (USB_Is_Ready() == 0)
    {
        return 1;
    }

    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return (hcdc->TxState != 0) ? 1 : 0;
}

void USB_Init(USB_Callback callback)
{
    USB_Manage_Object.Callback_Function = callback;
    USB_Manage_Object.Rx_Buffer_Active = UserRxBufferFS;
    USB_Manage_Object.Rx_Buffer_Ready = NULL;

    if (USB_Is_Ready() != 0)
    {
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USB_Manage_Object.Rx_Buffer_Active);
        (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }
}

/*
 * @brief  USB 发送数据函数
 * @param  data: 待发送数据指针
 * @param  len: 待发送长度
 * @retval USBD_OK / USBD_BUSY / USBD_FAIL
 */
uint8_t USB_SendData(const uint8_t *data, uint16_t len)
{
    uint8_t ret;
    uint32_t primask;

    if ((data == NULL) || (len == 0) || (len > USB_DATA_Send_MAX))
    {
        return USBD_FAIL;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (USB_Is_Tx_Busy() != 0)
    {
        if (primask == 0)
        {
            __enable_irq();
        }
        return USBD_BUSY;
    }

    memcpy(USB_Tx_Buffer, data, len);
    ret = CDC_Transmit_FS(USB_Tx_Buffer, len);

    if (primask == 0)
    {
        __enable_irq();
    }

    return ret;
}

/*
 * @brief  USB 发送字符串函数
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

    if (len > USB_DATA_Send_MAX)
    {
        return USBD_FAIL;
    }

    return USB_SendData((const uint8_t *)str, len);
}

void USB_Rx_Callback(uint8_t *buf, uint32_t len)
{
    if ((buf != NULL) && (len > 0))
    {
        USB_Manage_Object.Rx_Buffer_Ready = USB_Manage_Object.Rx_Buffer_Active;

        if (USB_Manage_Object.Rx_Buffer_Active == UserRxBufferFS)
        {
            USB_Manage_Object.Rx_Buffer_Active = USB_Manage_Object.Rx_Buffer_0;
        }
        else
        {
            USB_Manage_Object.Rx_Buffer_Active = UserRxBufferFS;
        }

        if (USB_Manage_Object.Callback_Function != NULL)
        {
            USB_Manage_Object.Callback_Function(USB_Manage_Object.Rx_Buffer_Ready, len);
        }
    }

    if (USB_Is_Ready() != 0)
    {
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USB_Manage_Object.Rx_Buffer_Active);
        (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }
}