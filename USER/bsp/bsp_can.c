#include "bsp_can.h"
#include "can.h"          /* CubeMX 的：hcan1 / hcan2 */
#include "main.h"

volatile uint32_t g_can_rx_cnt = 0U;          /* 总帧数 */
volatile uint32_t g_can_id_cnt[8] = {0};      /* 下标0~7 = 0x201~0x208 各收到多少帧 */

static bsp_can_rx_t s_rx_handler = 0;

uint8_t bsp_can_init(void)
{
    CAN_FilterTypeDef f = {0};

    f.FilterActivation     = ENABLE;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;              /* ID 全 0 */
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;              /* 掩码全 0 = 谁的都收 */
    f.FilterMaskIdLow      = 0x0000;
    f.FilterBank           = 0;                   /* CAN1 用 0 号 */
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.SlaveStartFilterBank = 14;                  /* CAN2 从 14 号起（现在没用，先写对） */

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) { return 0; }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)            { return 0; }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) { return 0; }
    return 1;
}

uint8_t bsp_can_send_frame(uint32_t id, const uint8_t *data, uint16_t len)
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox;

    if ((data == 0) || (len > 8U)) { return 0; }
    tx.StdId = id;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = (uint32_t)len;
    tx.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t *)data, &mailbox) != HAL_OK) { return 0; }
    return 1;
}

void bsp_can_register(bsp_can_rx_t handler) { s_rx_handler = handler; }

/* HAL 在 CAN1_RX0 中断里反过来调这个函数（名字是 HAL 定死的） */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t  data[8];
    uint16_t idx;

    if (hcan != &hcan1) { return; }
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) { return; }
    if (rx.IDE != CAN_ID_STD) { return; }

    g_can_rx_cnt++;                                   /* 中断里只做两件事：记账 + 转发 */
    idx = (uint16_t)(rx.StdId - 0x201U);
    if (idx < 8U) { g_can_id_cnt[idx]++; }
    if (s_rx_handler != 0) { s_rx_handler(rx.StdId, data, (uint16_t)rx.DLC); }
}