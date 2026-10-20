/* bsp_buzzer.h */
#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#include <stdint.h>

void bsp_buzzer_init(void);          /* 启动 PWM（此时静音） */
void bsp_buzzer_beep(uint16_t ms);   /* 响 ms 毫秒（阻塞式） */

#endif