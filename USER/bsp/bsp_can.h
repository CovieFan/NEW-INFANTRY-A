#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include <stdint.h>

/* 收到一帧时，按这个原型回调上层（谁注册谁实现） */
typedef void (*bsp_can_rx_t)(uint32_t id, const uint8_t *data, uint16_t len);

/* 诊断计数：步1 的现象就看这两个变量 */
extern volatile uint32_t g_can_rx_cnt;
extern volatile uint32_t g_can_id_cnt[8];

uint8_t bsp_can_init(void);
uint8_t bsp_can_send_frame(uint32_t id, const uint8_t *data, uint16_t len);
void    bsp_can_register(bsp_can_rx_t handler);

#endif

