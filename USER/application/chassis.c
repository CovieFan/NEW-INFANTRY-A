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


/* ================= 摇杆行程（实测标定）=================
   rc_full_pos[i] = 通道 i「读数为正」方向推到底的幅度
   rc_full_neg[i] = 通道 i「读数为负」方向推到底的幅度
   正常轴两个方向都是 660（DJI：1684-1024）。
   ⚠️ ch2 的某一侧实测只有 ~150 —— 在这里单独补偿（是"补偿"，不是"修复"）。 */
static const float rc_full_pos[4] = { 660.0f, 660.0f, 150.0f, 660.0f };  /* ch0~ch3 */
static const float rc_full_neg[4] = { 660.0f, 660.0f, 660.0f, 660.0f };

/* 取某个通道并归一化到 ±1（idx > 3 视为"该轴不用"，返回 0） */
static float rc_axis(uint8_t idx)
{
    int16_t ch;
    float   v;
    float   a;
    float   fs;

    switch (idx)                                   /* 直接取通道值（原 rc_ch() 已并入这里） */
    {
        case 0U: ch = remote.ch0; break;
        case 1U: ch = remote.ch1; break;
        case 2U: ch = remote.ch2; break;
        case 3U: ch = remote.ch3; break;
        default: return 0.0f;                      /* RC_CH_xxx = 0xFF -> 该轴恒 0 */
    }

    v = (float)ch;
    a = (v < 0.0f) ? -v : v;

    if (a < (float)RC_DEADBAND) { return 0.0f; }   /* 死区内 -> 0 */

    fs = (v >= 0.0f) ? rc_full_pos[idx] : rc_full_neg[idx];
    if (fs <= (float)RC_DEADBAND) { return 0.0f; } /* 防除零 */
    if (a > fs) { a = fs; }                        /* 超量程 -> 削平 */

    /* 死区外重映射：从死区边缘连续地长到 1（顺便修掉"刚出死区就跳"的问题） */
    a = (a - (float)RC_DEADBAND) / (fs - (float)RC_DEADBAND);
    return (ch < 0) ? -a : a;
}



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
    vx = rc_axis(RC_CH_VX) * (float)RC_SIGN_VX * (float)CHASSIS_FWD_CUR;
    vy = rc_axis(RC_CH_VY) * (float)RC_SIGN_VY * (float)CHASSIS_STRAFE_CUR;
    wz = rc_axis(RC_CH_WZ) * (float)RC_SIGN_WZ * (float)CHASSIS_SPIN_CUR;

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
    vx = rc_axis(RC_CH_VX) * (float)RC_SIGN_VX * CHASSIS_PID_MAX_RPM;
    vy = rc_axis(RC_CH_VY) * (float)RC_SIGN_VY * CHASSIS_PID_MAX_RPM;
    wz = rc_axis(RC_CH_WZ) * (float)RC_SIGN_WZ * CHASSIS_PID_MAX_RPM;

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


/* ---------------- 对外唯一入口 ---------------- */
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


