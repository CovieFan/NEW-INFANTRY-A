#ifndef __REMOTE_H
#define __REMOTE_H

#include <stdint.h>

void remote_init(void);        /* 挂上第一次接收 */
void remote_update(void);      /* 在线判定（步3b 才写） */


typedef struct
{
    int16_t  ch0;        /* 摇杆通道：约 -660 ~ +660，中位 ≈ 0 */
    int16_t  ch1;
    int16_t  ch2;
    int16_t  ch3;
    uint8_t  sw1;        /* 拨杆：1 / 2 / 3 */
    uint8_t  sw2;
    uint8_t  online;
    uint32_t last_ms;
} remote_t;

extern volatile remote_t remote;



/* 诊断变量：现象就看这几个 */
extern volatile uint16_t g_rc_bytes;      /* 最近一帧的字节数（应为 18） */
extern volatile uint32_t g_rc_frames;     /* 收到多少帧 */
extern volatile uint32_t g_rc_err_cnt;    /* 串口错误次数 */
extern volatile uint32_t g_rc_err_code;   /* 最近一次错误码 */

#endif