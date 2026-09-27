/* motor.h */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>


/* 一个电调的反馈（C620 手册 P14：ID = 0x200+电调ID，8 字节，全部大端） */
typedef struct
{
    int16_t  angle;        /* 转子机械角度 0~8191（对应 0~360°），会回绕 */
    int16_t  speed_rpm;    /* 转速，单位 RPM */
    int16_t  current;      /* 实际转矩电流（−16384~+16384 ↔ −20~+20A） */
    uint8_t  temp;         /* 温度 ℃ */
    uint8_t  online;       /* 1 = 至少收到过一次反馈 */
    uint32_t last_ms;      /* 最近一次收到反馈的时刻 */
} motor_t;

extern volatile motor_t motor_fb[4]; 

void motor_init(void);
void motor_send_current(const int16_t cur[4]);   /* cur[0]~cur[3] = 拨码 1~4 的电流 */

#endif

