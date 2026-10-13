/* gimbal.c —— 云台设备层（步5a：只收不发）
   分工：bsp_can 只管搬字节；这里才知道"0x205 是 yaw 的 6020"。 */
#include "gimbal.h"
#include "bsp_can.h"
#include "main.h"          /* HAL_GetTick() */
#include "remote.h"
#include "input.h"
#include "pid.h"
#include "bsp_imu.h"  

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
volatile uint8_t g_angle_rebased   = 0U;  /* 1 = 刚刚重新对齐过基准（控制层要跟着重贴目标）*/

volatile float   g_yaw_imu_angle  = 0.0f;  /* ★陀螺积分角 (rad)：相对大地 */
volatile   float   s_yaw_imu_lpf    = 0.0f;  /* ★60Hz 低通后的值 —— 外环反馈用它 */
static   uint8_t s_imu_ang_inited = 0U;


static uint8_t  s_angle_started  = 0U;

static void gimbal_angle_track(void)
{
    int16_t  raw = gimbal_fb[GIMBAL_AXIS_YAW].angle;
    int32_t  d;
    uint32_t now = HAL_GetTick();
    uint32_t last_rx = gimbal_fb[GIMBAL_AXIS_YAW].last_ms;   /* ★电机反馈帧自己的时间戳 */

    /* ① ★还没收到过任何一帧 6020 反馈 → 什么都不做！
          （此时 angle 还是初值 0：既不能当基准，也不能置 s_angle_started；
            否则真实反馈到来的那一帧 d = 真实值 - 0 是个大数 → 角度跳变 → 上电暴冲）*/
    if (last_rx == 0U)
    {
        return;
    }
    /* ① 建立基准（不累加）的条件：
          - 还没开始过；或
          - 从没收到过 6020 反馈（last_rx == 0，此时 angle 还是初值 0，不能当基准！）
          - 或 反馈超过 GIMBAL_ANGLE_GAP 没更新（掉线后回来，基准不可信）*/
    if ((s_angle_started == 0U) || (last_rx == 0U) ||
        ((now - last_rx) > GIMBAL_ANGLE_GAP))
    {
        g_yaw_angle_raw   = raw;
        g_yaw_angle_acc   = 0;
        g_yaw_angle_delta = 0;
        s_angle_started   = 1U;
        g_angle_rebased   = 1U;     /* 通知控制层：基准换了，目标要跟着贴 */
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
}


/* ============ 步5c-3：陀螺积分角（相对大地的绝对角）============
   目的：给外环一个"相对大地"的位置反馈 ⇒ 底盘自转时云台指向不变（自稳免费）
   公式：θ += ω × dt        ω = g_imu_gyro_yaw (rad/s)，dt = 1ms
   低通：60Hz 一阶，α = 2π×60×0.001 = 0.377（对齐 ARBATOS InsTask.c:998）
   ⚠️ 必须每 1ms 无条件执行 —— 所以调用点放在 gimbal_run()，不能塞进
      控制函数的提前 return 分支里（否则积分会断）*/
#define GIMBAL_IMU_DT        0.001f
#define GIMBAL_IMU_LPF_ALPHA 0.377f

static void gimbal_imu_angle_update(void)
{
    /* IMU 未在线 / 零偏标定还没完成 ⇒ 不积分（避免把标定期间的数积进去）*/
    if (g_imu_online == 0U)
    {
        return;
    }

    g_yaw_imu_angle += g_imu_gyro_yaw * GIMBAL_IMU_DT;

    if (s_imu_ang_inited == 0U)     /* 首次：低通直接对齐，别从 0 慢慢爬 */
    {
        s_yaw_imu_lpf    = g_yaw_imu_angle;
        s_imu_ang_inited = 1U;
        return;
    }
    s_yaw_imu_lpf += GIMBAL_IMU_LPF_ALPHA * (g_yaw_imu_angle - s_yaw_imu_lpf);
}

/* ============ 角度折回 [-PI, PI]（★对齐 ARBATOS，保命的一条）============
   ARBATOS：UserLib.h:83   #define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
            GimbalPid.c:33  pid->err = rad_format(set - get);
   作用：即使"目标"和"反馈"差了多圈，误差也只按"最近的那不到一圈"算
        ⇒ 误差永远有界（±PI）⇒ 不会失控
   ★ 为什么必需：纯陀螺积分角一定会漂；没有这一步，误差会一路涨上去
      ⇒ 上电自己转、而且越转越快（我们 10-08 就是这么挂的）*/
#define GIMBAL_PI 3.14159265358979f

static float gimbal_wrap_pi(float ang)
{
    while (ang >  GIMBAL_PI) { ang -= 2.0f * GIMBAL_PI; }
    while (ang < -GIMBAL_PI) { ang += 2.0f * GIMBAL_PI; }
    return ang;
}


/* ============ 步5c-2：YAW 控制（2026-10-07 改成「速度环 + 可切换外环」）============
   摇杆 → [模式] → ω_ref → [速度环] → 命令值 → 6020(0x1FF 槽0, CAN1)
     速控：ω_ref = 摇杆 × GIMBAL_YAW_SPEED_MAX
     位控：ω_ref = GIMBAL_YAW_KP_ANG × (θ_ref − θ)   ← 角度外环
   中科大 6.5 原话：「位控云台其实就是速控云台在时间上的累积」
   方向已实测确认（正电压 = acc 增大 / gyro_z 为正）⇒ 输出不需要 ×(-1) */

#define GIMBAL_YAW_SPEED_MODE  0U        /* ★1 = 速控（先验证这个）；0 = 位控 */

/* ---- 速度环（内外环共用的内环）---- */
#define GIMBAL_YAW_KP_W        500.0f   /* 命令值 / (rad/s)
                                            依据：ARBATOS 同款 6020 的 yaw_speed_pid.kp = 2000
                                            换算参考：746 = 1 倍"单位增益"，2000 = 2.68 倍 */
#define GIMBAL_YAW_KICK        0.0f      /* 起步电流（静摩擦补偿）：先 0，标定后再填
                                            标定法（中科大 6.6）：底盘放平，给 0.5 rad/s 目标，看反馈电流 */

/* ---- 速控：摇杆 → 目标角速度 ---- */
#define GIMBAL_YAW_SPEED_MAX   3.0f     /* 满杆目标角速度 = 400 °/s ÷ 57.3 */


/* ---- 位控：是否叠加"速度前馈"（中科大 6.5）----
   为什么需要：位控的纯角度环【必然】有"跟踪稳态误差"（推杆时云台会拖在后面），
   前馈把摇杆的角速度直接加到 ω_ref 上，误差为 0 时云台也在转 ⇒ 消掉滞后。
   0 = 关掉做对比，看前馈到底帮了多少 */
#define GIMBAL_YAW_OMEGA_FF_EN 0U



/* ---- 位控：角度外环 ---- */
#define GIMBAL_YAW_KP_ANG      39.0f     /* 单位 (rad/s)/rad；反馈 = 陀螺积分角 s_yaw_imu_lpf */
                                            /* ★2026-10-09 阶段1：严格照 ARBATOS（外环反馈=IMU角 + 误差折回±PI）；39 ≈ 它的 kp=30 */
#define GIMBAL_YAW_KI_ANG      0.0f      /* 先纯 P；位控的稳态误差用速度前馈消，不是靠 I（中科大 6.5）*/
#define GIMBAL_YAW_W_REF_LIM   20.0f     /* 期望角速度限幅 (rad/s) = 1146 °/s；防大误差时 ω_ref 爆掉 */
#define GIMBAL_YAW_INT_LIM     0.0f      /* KI_ANG=0，用不上 */

#define GIMBAL_YAW_TGT_RATE    0.0038f   /* 位控：摇杆积分成目标角度的速率（满杆 3.8 rad/s ≈ 218 °/s）*/
#define GIMBAL_YAW_TGT_LIMIT   10000.0f/* ★软限位；给这么大 ≈ 没限位 */

/* ---- 输出限幅（0x1FF 电压帧手册量程 ±25000）---- */
#define GIMBAL_YAW_OUT_LIM     20000.0f

/* ---- Watch 观测量 ---- */
volatile float   g_yaw_tgt       = 0.0f;  /* 位控：目标角度（连续多圈）*/
volatile float   g_yaw_err       = 0.0f;  /* 位控：角度误差（计数）*/
volatile float   g_yaw_omega_ref = 0.0f;  /* ★期望角速度 (rad/s)（两层共用的中间量）*/
volatile float   g_yaw_omega_ff  = 0.0f;  /* ★速度前馈项 (rad/s)（只有位控用）*/
volatile float   g_yaw_omega_fb  = 0.0f;  /* ★实际角速度 (rad/s)（陀螺，已去零偏）*/
volatile float   g_yaw_out       = 0.0f;  /* ★最终命令值（发出去的就是它）*/
volatile uint8_t g_yaw_hold      = 0U;    /* 位控：1 = 目标顶在软限位上 */

pid_t   s_pid_yaw;
static uint8_t s_yaw_ready = 0U;

static float gimbal_yaw_control(void)
{
    float axis;
    float out;

    /* ① 首次 / 掉线 → 目标贴着当前位置 + PID 复位（原逻辑不变）*/
    if (s_yaw_ready == 0U)
    {
        pid_init(&s_pid_yaw, GIMBAL_YAW_KP_ANG, GIMBAL_YAW_KI_ANG, 0.0f,
                 GIMBAL_YAW_W_REF_LIM, GIMBAL_YAW_INT_LIM);
        g_yaw_tgt       = s_yaw_imu_lpf;
        g_yaw_err       = 0.0f;
        g_yaw_omega_ref = 0.0f;
        g_yaw_omega_fb  = 0.0f;
        g_yaw_omega_ff  = 0.0f;
        g_yaw_out       = 0.0f;
        g_yaw_hold      = 0U;
        s_yaw_ready     = 1U;
        return 0.0f;
    }

    if (remote.online == 0U)
    {
        pid_reset(&s_pid_yaw);
        g_yaw_tgt       = s_yaw_imu_lpf;   /* 掉线期间目标跟着实际，重连不窜 */
        g_yaw_err       = 0.0f;
        g_yaw_omega_ref = 0.0f;
        g_yaw_omega_fb  = 0.0f;
        g_yaw_omega_ff  = 0.0f;
        g_yaw_out       = 0.0f;
        g_yaw_hold      = 0U;
        return 0.0f;
    }

    /* ★基准刚换过（上电首次收到反馈 / 掉线恢复）→ 目标也贴到新基准，防跳变暴冲 */
    if (g_angle_rebased != 0U)
    {
        g_angle_rebased = 0U;
        g_yaw_tgt       = s_yaw_imu_lpf;
        g_yaw_err       = 0.0f;
        g_yaw_omega_ref = 0.0f;
        pid_reset(&s_pid_yaw);
        return 0.0f;
    }

    /* ② 摇杆（input_axis 已归一化到 ±1，死区内 = 0）*/
    axis = input_axis(INPUT_AXIS_GIMBAL_YAW);

#if (GIMBAL_YAW_SPEED_MODE != 0U)
    /* 速控：摇杆直接给目标角速度；不积分角度 ⇒ 没有"冲过头"这回事 */
    g_yaw_omega_ref = axis * GIMBAL_YAW_SPEED_MAX;
    g_yaw_omega_ff  = 0.0f;              /* ★速控不用前馈 */
    g_yaw_hold      = 0U;
    g_yaw_err       = 0.0f;              /* 速控下没有"角度误差"这个概念 */
#else
    /* 位控：位置积分 → 角度外环给出 ω_ref；★再叠加"速度前馈"（中科大 6.5）
       跟踪时前馈兜住（不滞后），松手后外环锁位置（不滑行）*/
    g_yaw_tgt += axis * GIMBAL_YAW_TGT_RATE;
    g_yaw_hold = 0U;
    if (g_yaw_tgt >  GIMBAL_YAW_TGT_LIMIT) { g_yaw_tgt =  GIMBAL_YAW_TGT_LIMIT; g_yaw_hold = 1U; }
    if (g_yaw_tgt < -GIMBAL_YAW_TGT_LIMIT) { g_yaw_tgt = -GIMBAL_YAW_TGT_LIMIT; g_yaw_hold = 1U; }

    g_yaw_err = gimbal_wrap_pi(g_yaw_tgt - s_yaw_imu_lpf);

#if (GIMBAL_YAW_OMEGA_FF_EN != 0U)
    g_yaw_omega_ff = axis * GIMBAL_YAW_SPEED_MAX;      /* ★前馈 = 摇杆直接给的角速度 */
#else
    g_yaw_omega_ff = 0.0f;
#endif

    g_yaw_omega_ref = pid_calc(&s_pid_yaw, s_yaw_imu_lpf + g_yaw_err, s_yaw_imu_lpf)
                    + g_yaw_omega_ff;

    /* ω_ref 总限幅（前馈是加在外环输出【外面】的，必须再夹一次）*/
    if (g_yaw_omega_ref >  (float)GIMBAL_YAW_W_REF_LIM) { g_yaw_omega_ref =  (float)GIMBAL_YAW_W_REF_LIM; }
    if (g_yaw_omega_ref < -(float)GIMBAL_YAW_W_REF_LIM) { g_yaw_omega_ref = -(float)GIMBAL_YAW_W_REF_LIM; }
#endif

    /* ④ 速度环：KP_W × (期望角速度 − 实际角速度)
       反馈 = 投影到重力方向后的真 yaw 角速度（已减掉上电零偏）
       ⚠️ 陀螺掉线 ⇒ 速度环没有反馈 ⇒ 必须停输出（否则拿旧值当恒定偏置一直推云台）*/
    if (g_imu_online != 0U)
    {
        g_yaw_omega_fb = g_imu_gyro_yaw;
        out = GIMBAL_YAW_KP_W * (g_yaw_omega_ref - g_yaw_omega_fb);
    }
    else
    {
        g_yaw_omega_fb = 0.0f;
        out = 0.0f;
    }

    /* ⑤ 起步电流（静摩擦补偿）：按"期望角速度"方向给固定偏置
       依据：中科大 6.6 低配版 T = t·sgn(ω)；ARBATOS 的 GimbalYawKickCurrent 同款 */
    if (g_yaw_omega_ref > 0.0f)      { out += GIMBAL_YAW_KICK; }
    else if (g_yaw_omega_ref < 0.0f) { out -= GIMBAL_YAW_KICK; }

    /* ⑥ 限幅（pid_calc 只夹了外环输出，叠加起步电流后必须再夹一次）*/
    if (out >  (float)GIMBAL_YAW_OUT_LIM) { out =  (float)GIMBAL_YAW_OUT_LIM; }
    if (out < -(float)GIMBAL_YAW_OUT_LIM) { out = -(float)GIMBAL_YAW_OUT_LIM; }

    g_yaw_out = out;
    return out;
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
    gimbal_imu_angle_update();          /* ★新增：陀螺积分角（外环反馈）*/

    /* ========== YAW（6020 @ CAN1）：角度环 ========== */
#if (GIMBAL_RAW_TEST_ENABLE != 0U)
    out = gimbal_yaw_control();
#else
    out = 0.0f;
#endif
    gimbal_send_yaw((int16_t)out);     /* ★YAW 恢复输出（由 ch0 控制） */

    /* ========== PITCH（6623 @ CAN2）：角度环（闭环）========== */
#if (GIMBAL_RAW_TEST_ENABLE != 0U)
    out = gimbal_pitch_control();
#else
    out = 0.0f;
#endif
    gimbal_send_pitch((int16_t)out);
}
