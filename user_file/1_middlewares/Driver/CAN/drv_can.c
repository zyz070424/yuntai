#include "drv_can.h"

struct Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
struct Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

static uint8_t can_started[2] = {0};

/**
 * @brief 将 CAN 句柄映射到本地数组索引
 * @param hcan CAN 句柄
 * @retval 0:CAN1 1:CAN2 -1:无效句柄
 */
static int CAN_Get_Index(const CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return -1;
    }

    if (hcan->Instance == CAN1)
    {
        return 0;
    }

    if (hcan->Instance == CAN2)
    {
        return 1;
    }

    return -1;
}

/**
 * @brief 获取对应 CAN 的管理对象
 * @param hcan CAN 句柄
 * @retval 管理对象指针，失败返回 NULL
 */
static struct Struct_CAN_Manage_Object *CAN_Get_Manage_Object(const CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return NULL;
    }

    if (hcan->Instance == CAN1)
    {
        return &CAN1_Manage_Object;
    }

    if (hcan->Instance == CAN2)
    {
        return &CAN2_Manage_Object;
    }

    return NULL;
}

/**
 * @brief 将 Active 缓冲发布为 Ready 缓冲（仅在 Ready 为空时）
 * @param manage CAN 管理对象
 */
static void CAN_Publish_Active_To_Ready(struct Struct_CAN_Manage_Object *manage)
{
    if (manage == NULL)
    {
        return;
    }

    if (manage->Rx_Length_Active == 0)
    {
        return;
    }

    // 上一批 Ready 还未被处理，保留旧 Ready，新的先积压在 Active
    if (manage->Rx_Length_Ready != 0)
    {
        return;
    }

    manage->Rx_Buffer_Ready = manage->Rx_Buffer_Active;
    manage->Rx_Length_Ready = manage->Rx_Length_Active;

    if (manage->Rx_Buffer_Active == manage->Rx_Buffer_0)
    {
        manage->Rx_Buffer_Active = manage->Rx_Buffer_1;
    }
    else
    {
        manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
    }

    manage->Rx_Length_Active = 0;
}

/**
 * @brief 当 Ready 为空时，将 Active 提升为 Ready，避免数据滞留
 * @param manage CAN 管理对象
 */
static void CAN_Promote_Active_To_Ready_If_Needed(struct Struct_CAN_Manage_Object *manage)
{
    if (manage == NULL)
    {
        return;
    }

    if (manage->Rx_Length_Ready == 0 && manage->Rx_Length_Active > 0)
    {
        manage->Rx_Buffer_Ready = manage->Rx_Buffer_Active;
        manage->Rx_Length_Ready = manage->Rx_Length_Active;

        if (manage->Rx_Buffer_Active == manage->Rx_Buffer_0)
        {
            manage->Rx_Buffer_Active = manage->Rx_Buffer_1;
        }
        else
        {
            manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
        }

        manage->Rx_Length_Active = 0;
    }
}

/**
 * @brief  CAN 滤波器配置函数
 * @param  hcan: CAN句柄
 * @retval 无
 * @note  还没有写错误处理，还没想好会不会出问题
 */
void CAN_Filter_Config(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;

    if (hcan == NULL)
    {
        return;
    }

    memset(&sFilterConfig, 0, sizeof(sFilterConfig));

    // 双 CAN 模式下，滤波器 0~13 给 CAN1，14~27 给 CAN2
    sFilterConfig.SlaveStartFilterBank = 14;
    sFilterConfig.FilterBank = (hcan->Instance == CAN2) ? 14 : 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;      // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;     // 32 位滤波器
    sFilterConfig.FilterIdHigh = 0x0000;                   // 全通
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 统一进 FIFO0
    sFilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        // 处理错误
    }
}

/**
 * @brief  CAN 启动函数
 * @param  hcan: CAN句柄
 * @retval 无
 */
void CAN_Start(CAN_HandleTypeDef *hcan)
{
    int can_index = CAN_Get_Index(hcan);
    struct Struct_CAN_Manage_Object *manage = CAN_Get_Manage_Object(hcan);

    if (can_index < 0 || manage == NULL)
    {
        // 处理未知CAN实例的情况
        return;
    }

    // 初始化双缓冲指针（格式与 USB/USART 驱动保持一致）
    if (manage->Rx_Buffer_Active == NULL)
    {
        manage->hcan = hcan;
        manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
        manage->Rx_Length_Active = 0;

        manage->Rx_Buffer_Ready = manage->Rx_Buffer_1;
        manage->Rx_Length_Ready = 0;

        manage->Drop_Count = 0;
        manage->Notify_Task_Handle = NULL;
    }

    // 总线已启动则直接返回，防止重复调用 HAL_CAN_Start
    if (can_started[can_index] != 0)
    {
        return;
    }

    // 配置滤波器
    CAN_Filter_Config(hcan);

    // 启动CAN
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        // 处理错误
        return;
    }

    // 激活接收中断
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        // 处理错误
        return;
    }

    can_started[can_index] = 1;
}

/**
 * @brief  注册CAN接收就绪通知任务
 * @param  hcan: CAN句柄
 * @param  task_handle: 被通知任务句柄
 * @retval 无
 */
void CAN_Register_Notify_Task(CAN_HandleTypeDef *hcan, TaskHandle_t task_handle)
{
    struct Struct_CAN_Manage_Object *manage = CAN_Get_Manage_Object(hcan);

    if (manage == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    manage->Notify_Task_Handle = task_handle;
    taskEXIT_CRITICAL();
}

/** 
 * @brief  CAN 发送控制命令函数
 * @param  hcan: CAN句柄
 * @param  Send_id: 发送的CAN ID
 * @param  data: 待发送数据（8字节）
 * @retval 无
*/
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t Send_id, uint8_t *data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if (hcan == NULL || data == NULL)
    {
        return;
    }

    // 设置CAN ID
    tx_header.StdId = Send_id;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    // 发送消息
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
        // 处理错误
    }
}

/** 
 * @brief  CAN 接收回调函数
 * @param  hcan: CAN句柄
 * @retval 无
 */
void CAN_Receive_Callback(CAN_HandleTypeDef *hcan)
{
    struct Struct_CAN_Manage_Object *manage = CAN_Get_Manage_Object(hcan);
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (manage == NULL)
    {
        return;
    }

    // 中断里只做“搬运”：从硬件 FIFO 读出到 Active 缓冲
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if (manage->Rx_Length_Active < CAN_RX_BUFFER_SIZE)
        {
            uint16_t write_index = manage->Rx_Length_Active;
            manage->Rx_Buffer_Active[write_index].rx_header = rx_header;
            memcpy(manage->Rx_Buffer_Active[write_index].rx_data, rx_data, 8);
            manage->Rx_Length_Active++;
        }
        else
        {
            // Active 缓冲写满，统计丢帧
            manage->Drop_Count++;
        }
    }

    CAN_Publish_Active_To_Ready(manage);

    // 可选通知：Ready 就绪后唤醒任务，本来是使用任务通知的设计，但是这里用任务通知更方便，但是还是保留设计
    if (manage->Rx_Length_Ready > 0 && manage->Notify_Task_Handle != NULL)
    {
        vTaskNotifyGiveFromISR(manage->Notify_Task_Handle, &xHigherPriorityTaskWoken);
    }

    // 必要时立刻切到被唤醒的高优先级任务
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/** 
 * @brief  读取一帧CAN消息（先进先出）
 * @param  hcan: CAN句柄
 * @param  rx_message: 输出消息
 * @retval HAL_OK: 成功 HAL_ERROR: 无数据或参数错误
 */
HAL_StatusTypeDef CAN_ReadMessage(CAN_HandleTypeDef *hcan, CAN_RX_MESSAGE *rx_message)
{
    struct Struct_CAN_Manage_Object *manage = CAN_Get_Manage_Object(hcan);
    uint16_t i;

    if (manage == NULL || rx_message == NULL)
    {
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();

    CAN_Promote_Active_To_Ready_If_Needed(manage);

    if (manage->Rx_Length_Ready == 0)
    {
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }

    *rx_message = manage->Rx_Buffer_Ready[0];

    // 前移剩余数据，保持 Ready 区先进先出
    for (i = 1; i < manage->Rx_Length_Ready; i++)
    {
        manage->Rx_Buffer_Ready[i - 1] = manage->Rx_Buffer_Ready[i];
    }

    manage->Rx_Length_Ready--;

    taskEXIT_CRITICAL();

    return HAL_OK;
}

/** 
 * @brief  按标准ID读取一帧CAN消息（直接寻找）
 * @param  hcan: CAN句柄
 * @param  std_id: 目标标准ID
 * @param  rx_message: 输出消息
 * @retval HAL_OK: 成功 HAL_ERROR: 无匹配或参数错误
 */
HAL_StatusTypeDef CAN_ReadMessage_By_StdId(CAN_HandleTypeDef *hcan, uint32_t std_id, CAN_RX_MESSAGE *rx_message)
{
    struct Struct_CAN_Manage_Object *manage = CAN_Get_Manage_Object(hcan);
    uint16_t i;
    uint16_t j;

    if (manage == NULL || rx_message == NULL)
    {
        return HAL_ERROR;
    }
    //保护临界区
    taskENTER_CRITICAL();

    CAN_Promote_Active_To_Ready_If_Needed(manage);

    for (i = 0; i < manage->Rx_Length_Ready; i++)
    {
        if (manage->Rx_Buffer_Ready[i].rx_header.StdId == std_id)
        {
            *rx_message = manage->Rx_Buffer_Ready[i];

            // 删除匹配帧，压缩 Ready 区
            for (j = i + 1; j < manage->Rx_Length_Ready; j++)
            {
                manage->Rx_Buffer_Ready[j - 1] = manage->Rx_Buffer_Ready[j];
            }

            manage->Rx_Length_Ready--;

            taskEXIT_CRITICAL();
            return HAL_OK;
        }
    }

    taskEXIT_CRITICAL();

    return HAL_ERROR;
}

/*
 * @brief  CAN 接收中断回调函数
 * @param  hcan: CAN句柄
 * @retval 无
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Receive_Callback(hcan);
}
