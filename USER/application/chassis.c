/* ============================================================
 * chassis.c —— 底盘应用层
 * 职责：把"车该往哪走"变成 4 个轮子的电流。
 * 现状：4a 逐轮开环标定；以后放逆运动学 + 状态机 + 保护。
 * ============================================================ */
#include "chassis.h"
#include "motor.h"
#include "main.h"                       /* HAL_GetTick() */

#if (CHASSIS_OL_TEST != 0U)

/* ---------------- 标定参数 ---------------- */
#define OL_CUR        1000              /* 标定电流，满量程 ±16384（≈1.2A / 20A） */
#define OL_DELAY      3000U             /* 上电后等 3s 才动，给人时间松手 */
#define OL_ON         500U              /* 每个动作持续 500ms */
#define OL_GAP        1000U             /* 动作之间停 1000ms（这段时间发全 0） */
#define OL_SLOT       (OL_ON + OL_GAP)  /* 一格 = 1500ms */
#define OL_SLOT_END   5U                /* 一共 5 格：0~3 单轮，4 = 四个一起 */
#define OL_SEND_MS    1U                /* 发帧节拍 1ms ≈ 1kHz（电调硬要求） */

/* ★ 4a 实测标定结果：下标 0~3 = 0x201 FR / 0x202 FL / 0x203 RL / 0x204 RR */
/*   +1 = 该轮给正电流时是"往前转"；-1 = 相反 */
static const int8_t motor_dir[4] = { +1, -1, -1, +1 };

/* 每种整车动作在各轮上的"前进方向意图"(wf) */
/*             0x201FR 0x202FL 0x203RL 0x204RR */
static const int8_t act_wf[5][4] = {
    {    0,    0,    0,    0 },     /* 0 · 停              */
    {   +1,   +1,   +1,   +1 },     /* 1 · 前进            */
    {   -1,   -1,   -1,   -1 },     /* 2 · 后退            */
    {   +1,   -1,   -1,   +1 },     /* 3 · 左转(俯视逆时针) */
    {   -1,   +1,   +1,   -1 }      /* 4 · 右转(俯视顺时针) */
};

volatile uint8_t g_chassis_ol_slot = 0xFFU;
volatile int16_t g_chassis_ol_cur[4] = { 0, 0, 0, 0 };

static void chassis_ol_test_run(void)
{
    static uint32_t t0      = 0U;       /* 记录开始时刻 */
    static uint32_t tx_last = 0U;       /* 记录上次发 CAN 的时刻 */
    int16_t  cur[4] = { 0, 0, 0, 0 };
    uint32_t now = HAL_GetTick();
    uint32_t el;
    uint32_t slot;
    uint32_t ph;
    uint8_t  i;

    if (t0 == 0U) { t0 = now; }
    el = now - t0;                      /* 距开机的毫秒数 */

    if (el >= OL_DELAY)                 /* 前 3 秒什么都不做（照样发全 0） */
    {
        el  -= OL_DELAY;
        slot = (el / OL_SLOT) % OL_SLOT_END;
        ph   =  el % OL_SLOT;
        g_chassis_ol_slot = (uint8_t)slot;

        if (ph < OL_ON)
        {
            /* 意图(wf) × 方向修正(motor_dir) × 电流大小 = 实际发给电调的值 */
            for (i = 0U; i < 4U; i++)
            {
                cur[i] = (int16_t)(act_wf[slot][i] * motor_dir[i] * OL_CUR);
            }
        }
    }
    else
    {
        g_chassis_ol_slot = 0xFFU;
    }

    /* ★ 按 1ms 节拍持续发；全 0 也要发 —— "不发"不等于"停" */
    if ((now - tx_last) >= OL_SEND_MS)
    {
        tx_last = now;
        motor_send_current(cur);
        g_chassis_ol_cur[0] = cur[0];
        g_chassis_ol_cur[1] = cur[1];
        g_chassis_ol_cur[2] = cur[2];
        g_chassis_ol_cur[3] = cur[3];
    }
}
#endif /* CHASSIS_OL_TEST */

/* ---------------- 对外唯一入口 ---------------- */
void chassis_run(void)
{
#if (CHASSIS_OL_TEST != 0U)
    chassis_ol_test_run();
    return;                             /* 标定期间不跑正式底盘逻辑 */
#endif

    /* 以后写这里：逆运动学 + PID + 保护条件 */
}