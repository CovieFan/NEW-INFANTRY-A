/* ============================================================
 * chassis.c —— 底盘应用层
 * 职责：把"车该往哪走"变成 4 个轮子的电流。
 * 现状：4d 遥控开环 + 4c-1 单轮闭环测试
 * ============================================================ */
#include "chassis.h"
#include "motor.h"
#include "main.h"                       /* HAL_GetTick() */
#include "remote.h"
#include "pid.h"
#include "input.h"


/* ★ 4a 实测结果：下标 0~3 = 0x201 FR / 0x202 FL / 0x203 RL / 0x204 RR */
/*   +1 = 该轮给正电流时是"往前转"（左右镜像安装） */
const int8_t g_motor_dir[4] = { +1, -1, -1, +1 };

volatile int16_t g_dbg_cur[4] = { 0, 0, 0, 0 };

/* ================= 4d 遥控开环控制 ================= */
#if (CHASSIS_RC_ENABLE != 0U)


/* 给 Watch / 串口看的调试量 */
volatile float   g_dbg_vx  = 0.0f;
volatile float   g_dbg_vy  = 0.0f;
volatile float   g_dbg_wz  = 0.0f;

static void chassis_rc_run(void)
{
    static const int16_t zero[4] = { 0, 0, 0, 0 };
    float   vx;
    float   vy;
    float   wz;
    float   wf[4];
    int16_t cur[4] = { 0, 0, 0, 0 };
    uint8_t i;

    /* ① ★最重要的一行：遥控掉线 -> 立刻全 0 电流 */
    if (remote.online == 0U)
    {
        g_dbg_vx = 0.0f; g_dbg_vy = 0.0f; g_dbg_wz = 0.0f;
        for (i = 0U; i < 4U; i++) { g_dbg_cur[i] = 0; }
        motor_send_current(zero);
        return;
    }

    /* ② 摇杆 -> 三个"意图电流" */
    vx = input_axis(INPUT_AXIS_CHASSIS_X) * (float)CHASSIS_FWD_CUR;
    vy = input_axis(INPUT_AXIS_CHASSIS_Y) * (float)CHASSIS_STRAFE_CUR;
    wz = input_axis(INPUT_AXIS_CHASSIS_WZ) * (float)CHASSIS_SPIN_CUR;

    g_dbg_vx = vx; g_dbg_vy = vy; g_dbg_wz = wz;

    /* ③ 逆运动学（麦轮标准式，下标 0~3 = FR, FL, RL, RR）
          自转部分的符号已被 4a-2 实测确认过：左转 = 右侧+、左侧-  */
    wf[0] = vx + vy + wz;    /* FR */
    wf[1] = vx - vy - wz;    /* FL */
    wf[2] = vx + vy - wz;    /* RL */
    wf[3] = vx - vy + wz;    /* RR */

    /* ④ 硬限幅 + 方向修正 + 单轮掉线保护 */
    for (i = 0U; i < 4U; i++)
    {
        float c = wf[i];

        if (c >  (float)CHASSIS_CUR_MAX) { c =  (float)CHASSIS_CUR_MAX; }
        if (c < -(float)CHASSIS_CUR_MAX) { c = -(float)CHASSIS_CUR_MAX; }

        /* 该轮电调从没上线过 -> 不给电流（保护一个轮子死了还猛给的情况） */
        if (motor_fb[i].online == 0U) { c = 0.0f; }
        if ((motor_fb[i].last_ms == 0U) ||((HAL_GetTick() - motor_fb[i].last_ms) > 100U))
        {
            c = 0.0f;
        }

        cur[i] = (int16_t)(c * (float)g_motor_dir[i]);
        g_dbg_cur[i] = cur[i];
    }

    /* ⑤ 发出去。TIM6 已经给了 1kHz 节拍，这里不用再自己限速 */
    motor_send_current(cur);
}
#endif /* CHASSIS_RC_ENABLE */




/* ================= 4c-2 四轮转速闭环（遥控驱动） ================= */
#if (CHASSIS_PID_TEST != 0U)

volatile float   g_pid_tar[4] = { 0, 0, 0, 0 };   /* Watch：四轮目标（前进为正）*/
volatile float   g_pid_fb [4] = { 0, 0, 0, 0 };   /* Watch：四轮反馈（前进为正）*/
volatile float   g_pid_scale  = 1.0f;             /* Watch：等比缩放系数 */
volatile uint8_t g_pid_ok     = 0U;

static pid_t s_pid[4];

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

    if (g_pid_ok == 0U)                            /* 只初始化一次 */
    {
        for (i = 0U; i < 4U; i++)
        {
            pid_init(&s_pid[i], CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD,
                     CHASSIS_PID_OUT_LIM, CHASSIS_PID_INT_LIM);
        }
        g_pid_ok = 1U;
    }

    /* ① 遥控掉线 -> 目标全 0 + 清 PID（不然积分/状态会留残渣） */
    if (remote.online == 0U)
    {
        for (i = 0U; i < 4U; i++)
        {
            pid_reset(&s_pid[i]);
            g_pid_tar[i] = 0.0f;
            g_pid_fb [i] = (float)motor_fb[i].speed_rpm * (float)g_motor_dir[i];
            g_dbg_cur[i] = 0;
        }
        g_pid_scale = 0.0f;
        motor_send_current(zero);
        return;
    }

    /* ② 摇杆 -> 三个"意图转速"（单位：转子 rpm，前进为正）*/
    vx = input_axis(INPUT_AXIS_CHASSIS_X) * CHASSIS_PID_MAX_RPM;
    vy = input_axis(INPUT_AXIS_CHASSIS_Y) * CHASSIS_PID_MAX_RPM;
    wz = input_axis(INPUT_AXIS_CHASSIS_WZ) * CHASSIS_PID_MAX_RPM;

    /* ③ 逆运动学 -> 四轮目标转速（下标 0~3 = FR, FL, RL, RR）*/
    tgt[0] = vx + vy + wz;    /* FR */
    tgt[1] = vx - vy - wz;    /* FL */
    tgt[2] = vx + vy - wz;    /* RL */
    tgt[3] = vx - vy + wz;    /* RR */

    /* ④ ★等比缩放：超了就四轮一起缩 —— 保持运动方向不变
          （逐轮独立限幅会把方向扭歪，所以必须整体缩） */
    for (i = 0U; i < 4U; i++)
    {
        float a = (tgt[i] < 0.0f) ? -tgt[i] : tgt[i];
        if (a > maxabs) { maxabs = a; }
    }
    if (maxabs > CHASSIS_PID_MAX_RPM)
    {
        scale = CHASSIS_PID_MAX_RPM / maxabs;
        for (i = 0U; i < 4U; i++) { tgt[i] *= scale; }
    }
    g_pid_scale = scale;

    /* ⑤ 四轮 PID */
    for (i = 0U; i < 4U; i++)
    {
        float fb = (float)motor_fb[i].speed_rpm * (float)g_motor_dir[i];  /* 前进为正 */

        g_pid_tar[i] = tgt[i];
        g_pid_fb [i] = fb;

        /* 电调掉线 -> 清 PID + 不给电流 */
        if ((motor_fb[i].last_ms == 0U) ||
            ((HAL_GetTick() - motor_fb[i].last_ms) > 100U))
        {
            pid_reset(&s_pid[i]);
            cur[i]       = 0;
            g_dbg_cur[i] = 0;
            continue;
        }

        cur[i] = (int16_t)(pid_calc(&s_pid[i], tgt[i], fb) * (float)g_motor_dir[i]);
        g_dbg_cur[i] = cur[i];
    }

    motor_send_current(cur);
}

#endif /* CHASSIS_PID_TEST */


/* ---------------- 对外唯一入口！！！！！！！！！！！！！ ---------------- */
void chassis_run(void)
{
    remote_update();                 /* 1kHz 里顺手判遥控在线 */

#if (CHASSIS_PID_TEST != 0U)
    chassis_pid_run();          /* 4c-1：单轮闭环（优先于遥控） */
    return;
#endif

#if (CHASSIS_RC_ENABLE != 0U)
    chassis_rc_run();
    return;
#endif
    /* 两个开关都关着 -> 什么都不做（安全默认） */
}


