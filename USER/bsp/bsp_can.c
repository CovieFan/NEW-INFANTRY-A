#include "bsp_can.h"
#include "can.h"          /* CubeMX 的：hcan1 / hcan2 */
#include "main.h"

/* ---------------- 诊断计数 ---------------- */
volatile uint8_t  g_can1_ok          = 0U;
volatile uint8_t  g_can2_ok          = 0U;
volatile uint32_t g_can1_rx_cnt      = 0U;
volatile uint32_t g_can2_rx_cnt      = 0U;
volatile uint32_t g_can1_id_cnt[11]  = {0};
volatile uint32_t g_can2_id_cnt[11]  = {0};
volatile uint32_t g_can2_last_id     = 0U;
volatile uint8_t  g_can2_last_data[8]= {0};

/* ---------------- 接收者注册表 ---------------- */
typedef struct
{
    uint8_t      bus;          /* BSP_CAN_1 / BSP_CAN_2 */
    uint32_t     id_first;     /* 我要的 ID 段：[id_first, id_last] */
    uint32_t     id_last;
    bsp_can_rx_t handler;
} bsp_can_slot_t;

static bsp_can_slot_t s_slot[BSP_CAN_MAX_SLOT];
static uint8_t        s_slot_cnt = 0U;

/* ---------------- 内部：配过滤器 ---------------- */
static uint8_t can_config_filter(CAN_HandleTypeDef *hcan, uint32_t bank)
{
    CAN_FilterTypeDef f = {0};

    f.FilterActivation     = ENABLE;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;   /* 掩码全 0 = 谁的都收（软件里再筛） */
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterBank           = bank;     /* CAN1 = 0；CAN2 = 14 —— 绝对不能重叠 */
    f.SlaveStartFilterBank = 14U;      /* ★两次都要填 14：HAL 每次都会写 CAN1->FMR.CAN2SB */

    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) { return 0U; }
    return 1U;
}

static uint8_t can_start(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_Start(hcan) != HAL_OK) { return 0U; }
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) { return 0U; }
    return 1U;
}

uint8_t bsp_can_init(void)
{
    uint8_t ok = 1U;

    /* ★先把两条总线的过滤器都配好，再一起 Start
       （配过滤器会进 FINIT 模式，短暂停掉接收；放在 Start 之前就完全没影响） */
    if (can_config_filter(&hcan1,  0U) == 0U) { ok = 0U; }
    if (can_config_filter(&hcan2, 14U) == 0U) { ok = 0U; }

    g_can1_ok = can_start(&hcan1);
    g_can2_ok = can_start(&hcan2);

    return (uint8_t)(ok & g_can1_ok & g_can2_ok);
}

/* ---------------- 发送 ---------------- */
uint8_t bsp_can_send_frame(uint8_t bus, uint32_t id, const uint8_t *data, uint16_t len)
{
    CAN_HandleTypeDef  *hcan;
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t            mailbox;

    if ((data == 0) || (len > 8U)) { return 0U; }

    if      (bus == BSP_CAN_1) { hcan = &hcan1; }
    else if (bus == BSP_CAN_2) { hcan = &hcan2; }
    else                       { return 0U; }

    tx.StdId = id;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = (uint32_t)len;
    tx.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(hcan, &tx, (uint8_t *)data, &mailbox) != HAL_OK) { return 0U; }
    return 1U;
}

/* ---------------- 注册 ---------------- */
uint8_t bsp_can_register(uint8_t bus, uint32_t id_first, uint32_t id_last, bsp_can_rx_t handler)
{
    if (handler == 0)                              { return 0U; }
    if (id_first > id_last)                        { return 0U; }
    if ((bus != BSP_CAN_1) && (bus != BSP_CAN_2))  { return 0U; }
    if (s_slot_cnt >= BSP_CAN_MAX_SLOT)            { return 0U; }//看一下表满了吗

    s_slot[s_slot_cnt].bus      = bus;
    s_slot[s_slot_cnt].id_first = id_first;
    s_slot[s_slot_cnt].id_last  = id_last;
    s_slot[s_slot_cnt].handler  = handler;
    s_slot_cnt++;
    return 1U;
}

/* ---------------- 收到一帧：记账 + 分派（在中断里被调） ---------------- */
static void can_dispatch(uint8_t bus, uint32_t id, const uint8_t *data, uint16_t len)
{
    uint8_t  i;
    uint16_t k;

    /* ① 记账 */
    if (bus == BSP_CAN_1)
    {
        g_can1_rx_cnt++;
        if ((id >= 0x201U) && (id <= 0x20BU))
        {
            k = (uint16_t)(id - 0x201U);
            g_can1_id_cnt[k]++;
        }
    }
    else
    {
        g_can2_rx_cnt++;
        if ((id >= 0x201U) && (id <= 0x20BU))
        {
            k = (uint16_t)(id - 0x201U);
            g_can2_id_cnt[k]++;
        }
        g_can2_last_id = id;
        for (i = 0U; i < 8U; i++)
        {
            g_can2_last_data[i] = (i < len) ? data[i] : 0U;
        }
    }

    /* ② 分派：谁的 ID 段匹配就给谁 */
    for (i = 0U; i < s_slot_cnt; i++)
    {
        if (s_slot[i].bus != bus)     { continue; }
        if (id < s_slot[i].id_first)  { continue; }
        if (id > s_slot[i].id_last)   { continue; }
        s_slot[i].handler(id, data, len);
    }
}

/* HAL 在 CANx_RX0 中断里反过来调这个函数（名字是 HAL 定死的，两条总线共用） */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t  data[8];
    uint8_t  bus;

    if      (hcan->Instance == CAN1) { bus = BSP_CAN_1; }
    else if (hcan->Instance == CAN2) { bus = BSP_CAN_2; }
    else                             { return; }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) { return; }
    if (rx.IDE != CAN_ID_STD) { return; }

    can_dispatch(bus, rx.StdId, data, (uint16_t)rx.DLC);
}


