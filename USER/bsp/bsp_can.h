#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include <stdint.h>

/* ================= 总线编号 ================= */
#define BSP_CAN_1   1U      /* CAN1: PD0/PD1 */
#define BSP_CAN_2   2U      /* CAN2: PB12/PB13 */

/* 收到一帧时，按这个原型回调上层（谁注册谁实现） */
typedef void (*bsp_can_rx_t)(uint32_t id, const uint8_t *data, uint16_t len);

/* ---------------- 诊断计数（Watch 里看这些） ---------------- */
extern volatile uint8_t  g_can1_ok;           /* 1 = CAN1 启动成功 */
extern volatile uint8_t  g_can2_ok;           /* 1 = CAN2 启动成功 */
extern volatile uint32_t g_can1_rx_cnt;       /* CAN1 收到多少帧（原来叫 g_can_rx_cnt） */
extern volatile uint32_t g_can2_rx_cnt;       /* CAN2 收到多少帧  ★云台现象看它 */
extern volatile uint32_t g_can1_id_cnt[11];   /* CAN1: 下标 0~10 = 0x201~0x20B 各多少帧 */
extern volatile uint32_t g_can2_id_cnt[11];   /* CAN2: 同上 ★它告诉你云台电机是几号 */
extern volatile uint32_t g_can2_last_id;      /* CAN2 最近一帧的 ID */
extern volatile uint8_t  g_can2_last_data[8]; /* CAN2 最近一帧的 8 个字节 */

uint8_t bsp_can_init(void);

/* 发一帧：bus 填 BSP_CAN_1 或 BSP_CAN_2 */
uint8_t bsp_can_send_frame(uint8_t bus, uint32_t id, const uint8_t *data, uint16_t len);

/* 注册"我关心哪些 ID"：id_first ~ id_last 之间的帧交给 handler
   可注册多组，互不覆盖。返回 1 = 成功，0 = 参数非法 / 表满了 */
#define BSP_CAN_MAX_SLOT   8U
uint8_t bsp_can_register(uint8_t bus, uint32_t id_first, uint32_t id_last, bsp_can_rx_t handler);

#endif