#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/* ============ 云台电机（2026-09-29 定） ============
   YAW   = GM6020，挂 CAN1，反馈 0x205（拨码 ID 1），命令帧 0x1FF 槽位 0
   PITCH = 6623，  挂 CAN2，反馈 0x206（拨码 ID 2），命令帧 0x1FF 槽位 1
   依据：ARBATOS Robotconfig/INFANTRY-A/ConfigHardware.inc + MotorModelDb.c
         + 《6020电机.pdf》P6 拨码表 + 用户口述 */

#define GIMBAL_AXIS_YAW     0U
#define GIMBAL_AXIS_PITCH   1U
#define GIMBAL_AXIS_COUNT   2U

/* -------- 反馈 ID -------- */
#define GIMBAL_YAW_FB_ID    0x205U
#define GIMBAL_PITCH_FB_ID  0x206U

/* -------- 反馈帧格式（对应 ARBATOS 的两种 rx_format）--------
   RM_STD （6020 用）：角度@0 / 转速@2 / 实测电流@4 / 温度@6
   FMT_6623（6623 用）：角度@0 / 实测电流@2 / 设定电流回读@4 / 无转速 / 无温度
   ⚠️ 两者第 2-3 字节含义完全不同：一个是转速，一个是电流！ */
#define GIMBAL_FMT_RM_STD   0U
#define GIMBAL_FMT_6623     1U

typedef struct
{
    int16_t  angle;         /* 机械角度 0~8191（对应 0~360°），会回绕 */
    int16_t  speed_rpm;     /* ⚠️ has_speed==0 时恒为 0，不是真实转速 */
    int16_t  current_set;   /* 设定电流回读：RM_STD 用偏移 4；6623 用偏移 4 */
    int16_t  current_meas;  /* 实测电流：RM_STD 用偏移 4；6623 用偏移 2 */
    uint8_t  temp;          /* 温度℃；6623 无此字段 → 恒为 0 */
    uint8_t  has_speed;     /* 1 = 该电机有转速反馈 */
    uint8_t  has_temp;      /* 1 = 该电机有温度反馈 */
    uint8_t  online;        /* 1 = 至少收到过一次反馈 */
    uint32_t last_ms;       /* 最近一次收到反馈的时刻 */
    uint8_t  raw[8];        /* 最近一帧原始 8 字节，方便手工核对 */
} gimbal_motor_t;

extern volatile gimbal_motor_t gimbal_fb[GIMBAL_AXIS_COUNT];
extern volatile uint32_t       g_gimbal_rx_cnt[GIMBAL_AXIS_COUNT];

void gimbal_init(void);     /* 只注册接收；本步不发任何指令 */

#endif