/* chassis.c —— 底盘应用层：摇杆 → 四轮电流 */
#include "chassis.h"
#include "motor.h"
#include "main.h"                      
#include "remote.h"
#include "pid.h"
#include "input.h"
/* 下标 0~3 = 0x201 FR / 0x202 FL / 0x203 RL / 0x204 RR（实测） */
/* +1 = 正电流对应前进（左右镜像安装） */
const int8_t motor_dir[4] = { +1, -1, -1, +1 };

volatile int16_t dbg_cur[4] = { 0, 0, 0, 0 };
volatile chassis_mode_e chassis_mode = CHASSIS_MODE_PID_TEST;
volatile chassis_cfg_t chassis_cfg =
{
    .fwd_cur = 2500, .strafe_cur = 2500, .spin_cur = 2500, .cur_max = 4500,
    .spin_fixed_rpm = 1000.0f,
    .max_rpm = 3000.0f, .kp = 15.0f, .ki = 0.0f, .kd = 0.0f,
    .out_lim = 3000.0f, .int_lim = 3000.0f,
};

volatile float   dbg_vx  = 0.0f;
volatile float   dbg_vy  = 0.0f;
volatile float   dbg_wz  = 0.0f;

static void chassis_rc_run(void)
{
    static const int16_t zero[4] = { 0, 0, 0, 0 };
    float   vx;
    float   vy;
    float   wz;
    float   wf[4];
    int16_t cur[4] = { 0, 0, 0, 0 };
    uint8_t i;

    /* 掉线 -> 立即全 0 电流 */
    if (remote.online == 0U)
    {
        dbg_vx = 0.0f; dbg_vy = 0.0f; dbg_wz = 0.0f;
        for (i = 0U; i < 4U; i++) { dbg_cur[i] = 0; }
        motor_chassis_send_current(zero);
        return;
    }

    vx = input_axis(INPUT_AXIS_CHASSIS_X) * (float)chassis_cfg.fwd_cur;
    vy = input_axis(INPUT_AXIS_CHASSIS_Y) * (float)chassis_cfg.strafe_cur;
    wz = input_axis(INPUT_AXIS_CHASSIS_WZ) * (float)chassis_cfg.spin_cur;

    dbg_vx = vx; dbg_vy = vy; dbg_wz = wz;

    /* 麦轮逆运动学：0~3 = FR/FL/RL/RR
   自转符号已实测 */
    wf[0] = vx + vy + wz;    /* FR */
    wf[1] = vx - vy - wz;    /* FL */
    wf[2] = vx + vy - wz;    /* RL */
    wf[3] = vx - vy + wz;    /* RR */

    for (i = 0U; i < 4U; i++)
    {
        float c = wf[i];

        if (c >  (float)chassis_cfg.cur_max) { c =  (float)chassis_cfg.cur_max; }
        if (c < -(float)chassis_cfg.cur_max) { c = -(float)chassis_cfg.cur_max; }

        /* 该轮从未上线 -> 不给电流 */
        if (motor_chassis[i].fb.online == 0U) { c = 0.0f; }
        if ((motor_chassis[i].fb.last_ms == 0U) ||((HAL_GetTick() - motor_chassis[i].fb.last_ms) > 100U))
        {
            c = 0.0f;
        }

        cur[i] = (int16_t)(c * (float)motor_dir[i]);
        dbg_cur[i] = cur[i];
    }

    /* TIM6 已给 1kHz 节拍，这里不再限速 */
    motor_chassis_send_current(cur);
}



volatile float   pid_tar[4] = { 0, 0, 0, 0 };   /* Watch：四轮目标（前进为正）*/
volatile float   pid_fb [4] = { 0, 0, 0, 0 };   /* Watch：四轮反馈（前进为正）*/
volatile float   pid_scale  = 1.0f;             /* Watch：等比缩放系数 */
volatile uint8_t pid_ok     = 0U;

static pid_t chassis_pid[4];

static void chassis_pid_run(void)
{
    static const int16_t zero[4] = { 0, 0, 0, 0 };
    float   vx;
    float   vy;
    float   wz;
    float   tgt[4];
    int16_t cur[4] = { 0, 0, 0, 0 };
    float   maxabs = 0.0f;
    float   scale  = 1.0f;
    uint8_t i;

    if (pid_ok == 0U)
    {
        for (i = 0U; i < 4U; i++)
        {
            pid_init(&chassis_pid[i], chassis_cfg.kp, chassis_cfg.ki, chassis_cfg.kd,
                     chassis_cfg.out_lim, chassis_cfg.int_lim);
        }
        pid_ok = 1U;
    }

    for (i = 0U; i < 4U; i++)
    {
        chassis_pid[i].kp             = chassis_cfg.kp;
        chassis_pid[i].ki             = chassis_cfg.ki;
        chassis_pid[i].kd             = chassis_cfg.kd;
        chassis_pid[i].output_limit   = chassis_cfg.out_lim;
        chassis_pid[i].integral_limit = chassis_cfg.int_lim;
    }

    /* 掉线 -> 目标清零 + PID 复位，防积分残留 */
    if (remote.online == 0U)
    {
        for (i = 0U; i < 4U; i++)
        {
            pid_reset(&chassis_pid[i]);
            pid_tar[i] = 0.0f;
            pid_fb [i] = (float)motor_chassis[i].fb.speed_rpm * (float)motor_dir[i];
            dbg_cur[i] = 0;
        }
        pid_scale = 0.0f;
        motor_chassis_send_current(zero);
        return;
    }

    /* 摇杆 -> 目标转速（转子 rpm，前进为正） */
    vx = input_axis(INPUT_AXIS_CHASSIS_X) * chassis_cfg.max_rpm;
    vy = input_axis(INPUT_AXIS_CHASSIS_Y) * chassis_cfg.max_rpm;
    /* ★自转不再读摇杆：由拨杆中位决定（ch0 让给云台 yaw） */
    wz = (input_spin_on() != 0U) ? chassis_cfg.spin_fixed_rpm : 0.0f;

    /* 逆运动学 -> 四轮目标（0~3 = FR/FL/RL/RR） */
    tgt[0] = vx + vy + wz;    /* FR */
    tgt[1] = vx - vy - wz;    /* FL */
    tgt[2] = vx + vy - wz;    /* RL */
    tgt[3] = vx - vy + wz;    /* RR */

    /* 超速时四轮等比缩放：逐轮独立限幅会把方向扭歪 */
    for (i = 0U; i < 4U; i++)
    {
        float a = (tgt[i] < 0.0f) ? -tgt[i] : tgt[i];
        if (a > maxabs) { maxabs = a; }
    }
    if (maxabs > chassis_cfg.max_rpm)
    {
        scale = chassis_cfg.max_rpm / maxabs;
        for (i = 0U; i < 4U; i++) { tgt[i] *= scale; }
    }
    pid_scale = scale;

    for (i = 0U; i < 4U; i++)
    {
        float fb = (float)motor_chassis[i].fb.speed_rpm * (float)motor_dir[i];  /* 前进为正 */

        pid_tar[i] = tgt[i];
        pid_fb [i] = fb;

        /* 电调掉线 -> 清 PID + 不给电流 */
        if ((motor_chassis[i].fb.last_ms == 0U) ||
            ((HAL_GetTick() - motor_chassis[i].fb.last_ms) > 100U))
        {
            pid_reset(&chassis_pid[i]);
            cur[i]       = 0;
            dbg_cur[i] = 0;
            continue;
        }

        cur[i] = (int16_t)(pid_calc(&chassis_pid[i], tgt[i], fb) * (float)motor_dir[i]);
        dbg_cur[i] = cur[i];
    }

    motor_chassis_send_current(cur);
}


void chassis_run(void)
{
    switch (chassis_mode)
    {
    case CHASSIS_MODE_RC:
        chassis_rc_run();
        break;

    case CHASSIS_MODE_PID_TEST:
        chassis_pid_run();
        break;

    default:
        break;
    }
}
