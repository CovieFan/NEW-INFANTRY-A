#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef struct
{
    uint8_t speed_mode;    /* 1 = 速控；0 = 位控 */
    uint8_t omega_ff_en;   /* 位控：速度前馈开关 */
    float   kp_w;          /* 速度环 P：命令值 / (rad/s) */
    float   kick;          /* 起步电流（静摩擦补偿） */
    float   speed_max;     /* 速控：满杆目标角速度 (rad/s) */
    float   kp_ang;        /* 位控角度环 P：(rad/s)/rad */
    float   ki_ang;        /* 位控角度环 I */
    float   w_ref_lim;     /* 期望角速度限幅 (rad/s) */
    float   int_lim;       /* 角度环积分限幅 */
    float   tgt_rate;      /* 位控：摇杆积分速率 (rad/tick) */
    float   tgt_limit;     /* 位控：目标角度软限位 (rad) */
    float   out_lim;       /* 输出限幅（0x1FF 量程 +-25000） */
} gimbal_yaw_cfg_t;

typedef struct
{
    int16_t level;         /* 云台水平时 motor_gimbal[1].fb.angle 的实测值 */
    float   kp;
    float   ki;
    float   kd;
    float   out_lim;       /* 必须 <= motor_gimbal_cfg[1].out_max */
    float   int_lim;
    float   dir;           /* 方向系数 +-1 */
} gimbal_pitch_cfg_t;

typedef struct
{
    uint8_t enable;        /* 云台总开关：1 = 正常输出；0 = 强制 0 */
    gimbal_yaw_cfg_t   yaw;
    gimbal_pitch_cfg_t pitch;
} gimbal_cfg_t;

extern volatile gimbal_cfg_t gimbal_cfg;

void gimbal_run(void);

#endif
