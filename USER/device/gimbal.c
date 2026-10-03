/* gimbal.c —— 云台设备层（步5a：只收不发）
   分工：bsp_can 只管搬字节；这里才知道"0x205 是 yaw 的 6020"。 */
#include "gimbal.h"
#include "bsp_can.h"
#include "main.h"          /* HAL_GetTick() */
#include "remote.h"
#include "input.h"
#include "pid.h"

volatile gimbal_motor_t gimbal_fb[GIMBAL_AXIS_COUNT];
volatile uint32_t       g_gimbal_rx_cnt[GIMBAL_AXIS_COUNT];

/* 大端取 int16（和 motor.c 里一样，DJI 帧全部大端） */
static int16_t gimbal_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void gimbal_store_raw(uint8_t idx, const uint8_t *data)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++) { gimbal_fb[idx].raw[i] = data[i]; }
    gimbal_fb[idx].online  = 1U;
    gimbal_fb[idx].last_ms = HAL_GetTick();
    g_gimbal_rx_cnt[idx]++;
}

/* ---- YAW：GM6020，标准 RM 格式 ---- */
static void gimbal_yaw_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len < 8U)) { return; }
    if (id != GIMBAL_YAW_FB_ID)    { return; }

    gimbal_fb[GIMBAL_AXIS_YAW].angle        = gimbal_be16(&data[0]);
    gimbal_fb[GIMBAL_AXIS_YAW].speed_rpm    = gimbal_be16(&data[2]);  /* 转速 */
    gimbal_fb[GIMBAL_AXIS_YAW].current_meas = gimbal_be16(&data[4]);  /* 实测电流 */
    gimbal_fb[GIMBAL_AXIS_YAW].current_set  = gimbal_be16(&data[4]);  /* 同一格 */
    gimbal_fb[GIMBAL_AXIS_YAW].temp         = data[6];                /* 温度 */

    gimbal_store_raw(GIMBAL_AXIS_YAW, data);
}

/* ---- PITCH：6623，专用格式（⚠️ 第 2-3 字节是电流，不是转速！）---- */
static void gimbal_pitch_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len < 8U)) { return; }
    if (id != GIMBAL_PITCH_FB_ID)  { return; }

    gimbal_fb[GIMBAL_AXIS_PITCH].angle        = gimbal_be16(&data[0]);  /* 角度也在偏移 0 */
    gimbal_fb[GIMBAL_AXIS_PITCH].current_meas = gimbal_be16(&data[2]);  /* 偏移2 = 实测电流 */
    gimbal_fb[GIMBAL_AXIS_PITCH].current_set  = gimbal_be16(&data[4]);  /* 偏移4 = 设定电流回读 */
    /* ⚠️ 下面两行是"故意置 0"，因为这个型号的帧里根本没有这两个字段 */
    gimbal_fb[GIMBAL_AXIS_PITCH].speed_rpm    = 0;
    gimbal_fb[GIMBAL_AXIS_PITCH].temp         = 0U;

    gimbal_store_raw(GIMBAL_AXIS_PITCH, data);
}

/* ---- PITCH：把命令值塞进 0x1FF 的槽位 1，发到 CAN2 ---- */
volatile int16_t  g_gimbal_tx_pitch      = 0;
volatile uint32_t g_gimbal_tx_pitch_ok   = 0U;
volatile uint32_t g_gimbal_tx_pitch_fail = 0U;

void gimbal_send_pitch(int16_t cmd)
{
    uint8_t tx[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t off   = (uint8_t)(GIMBAL_PITCH_CMD_SLOT * 2U);   /* 槽位1 → 字节 2-3 */

    if (cmd >  (int16_t)GIMBAL_PITCH_CMD_MAX) { cmd =  (int16_t)GIMBAL_PITCH_CMD_MAX; }
    if (cmd < -(int16_t)GIMBAL_PITCH_CMD_MAX) { cmd = -(int16_t)GIMBAL_PITCH_CMD_MAX; }

    tx[off]      = (uint8_t)((uint16_t)cmd >> 8U);        /* 大端 */
    tx[off + 1U] = (uint8_t)((uint16_t)cmd & 0xFFU);

    if (bsp_can_send_frame(BSP_CAN_2, GIMBAL_PITCH_CMD_ID, tx, 8U) != 0U)  /* ★ CAN2 */
    {
        g_gimbal_tx_pitch_ok++;
    }
    else
    {
        g_gimbal_tx_pitch_fail++;
    }

    g_gimbal_tx_pitch = cmd;
}
void gimbal_init(void)
{
    uint8_t i;

    for (i = 0U; i < GIMBAL_AXIS_COUNT; i++)
    {
        gimbal_fb[i].angle        = 0;
        gimbal_fb[i].speed_rpm    = 0;
        gimbal_fb[i].current_set  = 0;
        gimbal_fb[i].current_meas = 0;
        gimbal_fb[i].temp         = 0U;
        gimbal_fb[i].has_speed    = 0U;
        gimbal_fb[i].has_temp     = 0U;
        gimbal_fb[i].online       = 0U;
        gimbal_fb[i].last_ms      = 0U;
        g_gimbal_rx_cnt[i]        = 0U;
    }

    /* 这两个电机的能力标注（供上层判断"能不能做速度环"） */
    gimbal_fb[GIMBAL_AXIS_YAW].has_speed = 1U;   /* 6020：有转速 */
    gimbal_fb[GIMBAL_AXIS_YAW].has_temp  = 1U;
    /* PITCH 保持 0：6623 既没有转速也没有温度 */

    /* ★★ 只用"注册接收"，本步不发任何 CAN 指令 ★★
       一次注册同时验证三件事：① 双总线改造生效 ② 6020 的 0x205 没被 motor_rx 吞掉 ③ 6623 在 CAN2 上活着 */
    (void)bsp_can_register(BSP_CAN_1, GIMBAL_YAW_FB_ID,   GIMBAL_YAW_FB_ID,   gimbal_yaw_rx);
    (void)bsp_can_register(BSP_CAN_2, GIMBAL_PITCH_FB_ID, GIMBAL_PITCH_FB_ID, gimbal_pitch_rx);
}


/* ============ 步5c-1：把"0~8191 的机械角"变成"连续多圈角度" ============
   只算、不控制 —— 本步零风险，先看变量对不对。
   为什么要做：直接拿会回绕的角度做 PID，在回绕点会算出 ±8192 的巨大误差
              → 云台会狂转。所以先算出"单调连续的累计角度"。 */

#define GIMBAL_ENC_RANGE   8192L    /* 13 位编码器：0~8191 */
#define GIMBAL_ENC_HALF    4096L    /* 半圈，用来判"是否发生回绕" */
#define GIMBAL_ANGLE_GAP   50U      /* 超过这么久没新帧 → 重新对齐（防掉线后跳变） */

volatile int32_t g_yaw_angle_acc   = 0;   /* ★累计多圈角度（连续，不会跳） */
volatile int16_t g_yaw_angle_raw   = 0;   /* 本帧原始角度 0~8191 */
volatile int16_t g_yaw_angle_delta = 0;   /* 本帧增量（正常只有几十以内） */

static uint32_t s_angle_last_ms  = 0U;
static uint8_t  s_angle_started  = 0U;

static void gimbal_angle_track(void)
{
    int16_t  raw = gimbal_fb[GIMBAL_AXIS_YAW].angle;
    int32_t  d;
    uint32_t now = HAL_GetTick();

    /* ① 首次上电 / 掉线过 → 只对齐基准，不累加（否则第一帧会算出垃圾增量） */
    if ((s_angle_started == 0U) || ((now - s_angle_last_ms) > GIMBAL_ANGLE_GAP))
    {
        g_yaw_angle_raw   = raw;
        g_yaw_angle_acc   = 0;
        g_yaw_angle_delta = 0;
        s_angle_started   = 1U;
        s_angle_last_ms   = now;
        return;
    }

    /* ② 算"本帧转了多少" */
    d = (int32_t)raw - (int32_t)g_yaw_angle_raw;

    /* ③ ★回绕修正：1ms 内云台不可能真的转半圈以上 → 超过半圈的一定是回绕 */
    if (d >  GIMBAL_ENC_HALF) { d -= GIMBAL_ENC_RANGE; }
    if (d < -GIMBAL_ENC_HALF) { d += GIMBAL_ENC_RANGE; }

    g_yaw_angle_delta  = (int16_t)d;
    g_yaw_angle_raw    = raw;
    g_yaw_angle_acc   += d;          /* ★累加 → 得到连续角度 */

    s_angle_last_ms = now;
}

/* ============ 步5c-2：YAW 角度环（闭环） ============
   摇杆 → 目标角度（增量式，照 ARBATOS 的 add_yaw）
   PID  → 电压
   松手 → 目标不变 → 云台锁住；到限位 → 顶住
   方向已实测确认（正电压 = acc 增大）⇒ PID 输出不需要 ×(-1) */

#define GIMBAL_YAW_KP        3.0f      /* ★先保守，看现象再调（err 单位=编码器单位）*/
#define GIMBAL_YAW_KI        0.0f
#define GIMBAL_YAW_KD        0.0f
#define GIMBAL_YAW_OUT_LIM   4000.0f   /* ★第一版故意压小：万一方向反了也只是"慢慢转"，不会飞 */
#define GIMBAL_YAW_INT_LIM   0.0f      /* KI=0，用不上 */

#define GIMBAL_YAW_TGT_RATE  2.5f      /* 满杆目标角速度 = 1500 单位/秒（实测上限 2000+，留余量）*/
#define GIMBAL_YAW_TGT_LIMIT 4096.0f   /* ★软限位 ≈ ±90°（第一版保守；测出行程后再放宽）*/

volatile float   g_yaw_tgt     = 0.0f;   /* 目标角度（连续多圈）*/
volatile float   g_yaw_err     = 0.0f;   /* 误差 = 目标 − 反馈 */
volatile float   g_yaw_pid_out = 0.0f;   /* PID 算出来的电压（本步会真的发出去）*/
volatile uint8_t g_yaw_hold    = 0U;     /* 1 = 目标已经顶在软限位上 */

static pid_t   s_pid_yaw;
static uint8_t s_yaw_ready = 0U;

static float gimbal_yaw_control(void)
{
    float axis;

    /* ① ★首次 / 掉线 → 目标贴着当前位置 + PID 复位
       （防"上电瞬间窜向目标" + 防"掉线期间积分涨满、重连窜一下"）*/
    if (s_yaw_ready == 0U)
    {
        pid_init(&s_pid_yaw, GIMBAL_YAW_KP, GIMBAL_YAW_KI, GIMBAL_YAW_KD,
                 GIMBAL_YAW_OUT_LIM, GIMBAL_YAW_INT_LIM);
        g_yaw_tgt     = (float)g_yaw_angle_acc;
        g_yaw_err     = 0.0f;
        g_yaw_pid_out = 0.0f;
        g_yaw_hold    = 0U;
        s_yaw_ready   = 1U;
        return 0.0f;
    }

    if (remote.online == 0U)
    {
        pid_reset(&s_pid_yaw);
        g_yaw_tgt     = (float)g_yaw_angle_acc;   /* 掉线期间目标跟着实际，重连不窜 */
        g_yaw_err     = 0.0f;
        g_yaw_pid_out = 0.0f;
        g_yaw_hold    = 0U;
        return 0.0f;
    }

    /* ② 摇杆增量 → 目标角度（摇杆表达的是"目标转多快"，不是"目标在哪"）*/
    axis = input_axis(INPUT_AXIS_GIMBAL_YAW);
    g_yaw_tgt += axis * GIMBAL_YAW_TGT_RATE;

    /* ③ ★软限位：夹住目标（防"按住摇杆目标无限增长"）*/
    g_yaw_hold = 0U;
    if (g_yaw_tgt >  GIMBAL_YAW_TGT_LIMIT) { g_yaw_tgt =  GIMBAL_YAW_TGT_LIMIT; g_yaw_hold = 1U; }
    if (g_yaw_tgt < -GIMBAL_YAW_TGT_LIMIT) { g_yaw_tgt = -GIMBAL_YAW_TGT_LIMIT; g_yaw_hold = 1U; }

    /* ④ 角度环：目标和反馈都已经是"连续多圈角度"，可以直接相减 */
    g_yaw_err     = g_yaw_tgt - (float)g_yaw_angle_acc;
    g_yaw_pid_out = pid_calc(&s_pid_yaw, g_yaw_tgt, (float)g_yaw_angle_acc);

    return g_yaw_pid_out;
}

/* ============ 步5d-2：PITCH 角度环（闭环）============
   基准 = 固定的"水平值"：上电时云台在任意位置，都会自己往水平靠。
   ⚠️ 实测：往下 angle 减小 / 往上 angle 增大 ⇒ 不取反（正命令 = 往上） */

#define GIMBAL_PITCH_LEVEL      6000    /* ★实测：云台水平时 gimbal_fb[1].angle 的值 */
#define GIMBAL_PITCH_ENC_RANGE  8192L   /* 13 位编码器 */
#define GIMBAL_PITCH_ENC_HALF   4096L   /* 半圈，用于回绕修正 */

#define GIMBAL_PITCH_KP         12.0f    /* ★先保守；err 单位 = 编码器计数 */
#define GIMBAL_PITCH_KI         0.01f    /* 先纯 P；pitch 有重力，最后要加（见上面表格） */
#define GIMBAL_PITCH_KD         0.0f
#define GIMBAL_PITCH_OUT_LIM    2000.0f /* ★必须 = GIMBAL_PITCH_CMD_MAX（手册量程 ±5000） */
#define GIMBAL_PITCH_INT_LIM    200000.0f    /* KI=0 用不上；加 KI 时改 200000.0f */

#define GIMBAL_PITCH_TGT_RATE   2.0f    /* 满杆目标角速度 = 2000 计数/秒（≈88°/s） */
#define GIMBAL_PITCH_TGT_LIMIT  800.0f  /* 软限位：相对水平 ±800 计数（≈±35°） */

#define GIMBAL_PITCH_DIR        (-1.0f)//改方向


volatile float   g_pitch_tgt     = 0.0f;   /* 目标；0 = 水平 */
volatile float   g_pitch_fb      = 0.0f;   /* 反馈；正 = 水平上方 */
volatile float   g_pitch_err     = 0.0f;   /* 误差 = 目标 − 反馈 */
volatile float   g_pitch_pid_out = 0.0f;   /* PID 输出（= 要发给电调的电流命令）*/
volatile uint8_t g_pitch_hold    = 0U;     /* 1 = 目标顶在软限位上 */

pid_t   s_pid_pitch;
static uint8_t s_pitch_ready = 0U;


/* 把"原始角 0~8191"换算成"相对水平的偏差"：正 = 水平上方
   为什么要有回绕修正：万一 angle 从 0 跳到 8191，直接相减会得到 ±8192 的假误差 */
static float gimbal_pitch_fb(void)
{
    int32_t d = (int32_t)gimbal_fb[GIMBAL_AXIS_PITCH].angle - (int32_t)GIMBAL_PITCH_LEVEL;

    if (d >  GIMBAL_PITCH_ENC_HALF) { d -= GIMBAL_PITCH_ENC_RANGE; }
    if (d < -GIMBAL_PITCH_ENC_HALF) { d += GIMBAL_PITCH_ENC_RANGE; }

    return (float)d;
}

/* 步5d-3：只做"松手自稳" —— 目标固定为水平，不接摇杆 */
static float gimbal_pitch_control(void)
{
    /* ① 首次上电 → PID 复位，本帧先不发电流 */
    if (s_pitch_ready == 0U)
    {
        pid_init(&s_pid_pitch, GIMBAL_PITCH_KP, GIMBAL_PITCH_KI, GIMBAL_PITCH_KD,
                 GIMBAL_PITCH_OUT_LIM, GIMBAL_PITCH_INT_LIM);
        g_pitch_tgt     = 0.0f;
        g_pitch_fb      = gimbal_pitch_fb();
        g_pitch_err     = 0.0f;
        g_pitch_pid_out = 0.0f;
        s_pitch_ready   = 1U;
        return 0.0f;
    }

    /* ② 目标恒 = 0 = 水平；反馈以"水平 = 0、上方为正"为单位 → 直接相减，不取反 */
    g_pitch_tgt     = 0.0f;
    g_pitch_fb      = gimbal_pitch_fb();
    g_pitch_err     = g_pitch_tgt - g_pitch_fb;
    g_pitch_pid_out = pid_calc(&s_pid_pitch, g_pitch_tgt, g_pitch_fb) * GIMBAL_PITCH_DIR;

    return g_pitch_pid_out;
}

/* ================= 步5b-1：YAW 开环（RAW 模式） =================
   ⚠️ 本步只验证"云台能动"，不是最终形态。5c 会升级成角度环。 */

volatile int16_t  g_gimbal_tx_yaw  = 0;
volatile uint32_t g_gimbal_tx_ok   = 0U;
volatile uint32_t g_gimbal_tx_fail = 0U;

/* ---- 把命令值塞进 0x1FF 的槽位 0，发到 CAN1 ----
   一帧 8 字节 = 4 个槽位（每个 int16 大端）。6020 只认自己那 2 个字节。 */
void gimbal_send_yaw(int16_t cmd)
{
    uint8_t tx[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t off   = (uint8_t)(GIMBAL_YAW_CMD_SLOT * 2U);

    if (cmd >  (int16_t)GIMBAL_YAW_CMD_MAX) { cmd =  (int16_t)GIMBAL_YAW_CMD_MAX; }
    if (cmd < -(int16_t)GIMBAL_YAW_CMD_MAX) { cmd = -(int16_t)GIMBAL_YAW_CMD_MAX; }

    tx[off]      = (uint8_t)((uint16_t)cmd >> 8U);        /* 大端：高字节在前 */
    tx[off + 1U] = (uint8_t)((uint16_t)cmd & 0xFFU);

    if (bsp_can_send_frame(BSP_CAN_1, GIMBAL_YAW_CMD_ID, tx, 8U) != 0U)
    {
        g_gimbal_tx_ok++;
    }
    else
    {
        g_gimbal_tx_fail++;       /* ⭐ 以前这一路失败是"静默"的，现在看得见 */
    }

    g_gimbal_tx_yaw = cmd;
}

void gimbal_run(void)
{
    float out;

    gimbal_angle_track();               /* 5c-1：yaw 的连续多圈角度（本步不动） */

    /* ========== YAW（6020 @ CAN1）：角度环 ========== */
#if (GIMBAL_RAW_TEST_ENABLE != 0U)
    out = gimbal_yaw_control();
#else
    out = 0.0f;
#endif
    //gimbal_send_yaw((int16_t)out);    /* ★YAW轴不动 */

    /* ========== PITCH（6623 @ CAN2）：角度环（闭环）========== */
#if (GIMBAL_RAW_TEST_ENABLE != 0U)
    out = gimbal_pitch_control();
#else
    out = 0.0f;
#endif
    gimbal_send_pitch((int16_t)out);
}