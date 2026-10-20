
#include "gimbal.h"
#include "motor.h"
#include "main.h"         
#include "remote.h"
#include "input.h"
#include "pid.h"
#include "bsp_imu.h"  

#define GIMBAL_ENC_RANGE   8192L    /* 13 位编码器：0~8191 */
#define GIMBAL_ENC_HALF    4096L    /* 半圈，用来判"是否发生回绕" */
#define GIMBAL_ANGLE_GAP   50U      /* 超过这么久没新帧 → 重新对齐（防掉线后跳变） */

volatile int32_t yaw_angle_acc   = 0;   /* 6020电机编码器累积的值，相对于底盘。这个数据是准的，因此在底盘不动的情况下可以当作漂移指示器来用 */
volatile int16_t yaw_angle_raw   = 0;   /* 本帧原始角度 0~8191 */
volatile int16_t yaw_angle_delta = 0;   /* 编码器值增量 */
volatile uint8_t angle_rebased   = 0U;  /* 1 = 刚刚重新对齐过基准（控制层要跟着重贴目标）*/

volatile float   yaw_imu_angle  = 0.0f;  /* IMU的积分角，相对于大地 */
volatile float   yaw_imu_lpf    = 0.0f;  /* 60Hz低通滤波后的值 	low-pass filter，低通滤波*/
volatile float   yaw_angle_deg   = 0.0f;  /*YAW6020电机编码器反馈的实际度数 */
volatile float   yaw_imu_deg     = 0.0f;  /* 外环IMU反馈的实际度数 */
static   uint8_t imu_ang_inited = 0U;


static uint8_t  angle_started  = 0U;

static void gimbal_angle_track(void)//云台编码器反馈值累积
/*加了一堆保护，本质其实就是把d累加到g_yaw_angle_acc，其中d本质是delta，是两次RAW之差*/
{
    int16_t  raw = motor_gimbal_fb[GIMBAL_AXIS_YAW].angle;
    int32_t  d;
    uint32_t now = HAL_GetTick();
    uint32_t last_rx = motor_gimbal_fb[GIMBAL_AXIS_YAW].last_ms;   /* ★电机反馈帧自己的时间戳 */

    if (last_rx == 0U)/*还没收到反馈，啥也不干*/
    {
        return;
    }
    /*  没开始/电机掉线/没收到电机反馈值不累加*/
    if ((angle_started == 0U) || (last_rx == 0U) ||
        ((now - last_rx) > GIMBAL_ANGLE_GAP))
    {
        yaw_angle_raw   = raw;
        yaw_angle_acc   = 0;
        yaw_angle_delta = 0;
        angle_started   = 1U;
        angle_rebased   = 1U;     /* 通知控制层：基准换了，目标要跟着贴 */
        return;
    }

    /* d=delta 本质是两次RAW之差，也就是编码器数据之差 */
    d = (int32_t)raw - (int32_t)yaw_angle_raw;

    /* 回绕修正，转满一圈发现d为负数时候直接加或减一个周期即可 */
    if (d >  GIMBAL_ENC_HALF) { d -= GIMBAL_ENC_RANGE; }
    if (d < -GIMBAL_ENC_HALF) { d += GIMBAL_ENC_RANGE; }

    yaw_angle_delta  = (int16_t)d;
    yaw_angle_raw    = raw;
    yaw_angle_acc   += d;          /* 编码器值累加 */
}

#define GIMBAL_IMU_DT        0.001f//积分步长，是由TIM6 1KHz决定的，每秒一千次
#define GIMBAL_IMU_LPF_ALPHA 0.377f//一阶低通滤波系数alpha

static void gimbal_imu_angle_update(void)//陀螺仪角度积分
{
    /* IMU 不在线 / 零偏标定还没完成 ⇒ 直接退出*/
    if (imu_online == 0U)
    {
        return;
    }

    yaw_imu_angle += imu_gyro_yaw * GIMBAL_IMU_DT;/* g_imu_gyro_yaw是陀螺仪的角速度，GIMBAL_IMU_DT是时间间隔 这一步是角速度积分得到角度*/
/*g_yaw_imu_angle是陀螺仪积分出来的原始角度，实际使用的时候需要进行滤波处理*/
    if (imu_ang_inited == 0U)     /* 首次：低通直接对齐，不从 0 慢慢爬 */
    {
        yaw_imu_lpf    = yaw_imu_angle;
        imu_ang_inited = 1U;
        return;
    }
    yaw_imu_lpf += GIMBAL_IMU_LPF_ALPHA * (yaw_imu_angle - yaw_imu_lpf);//一阶低通滤波
}

/*防止南辕北辙，走最近的路*/
#define GIMBAL_PI 3.14159265358979f
static float gimbal_wrap_pi(float ang)//保障角度误差在正负pai之内
{
    while (ang >  GIMBAL_PI) { ang -= 2.0f * GIMBAL_PI; }
    while (ang < -GIMBAL_PI) { ang += 2.0f * GIMBAL_PI; }
    return ang;
}

#define GIMBAL_YAW_SPEED_MODE  0U        /* 云台YAW的控制模式    1 = 速控；0 = 位控 */

/* ---- 速度环---- */
#define GIMBAL_YAW_KP_W        500.0f   /* 命令值 / (rad/s)*/
#define GIMBAL_YAW_KICK        0.0f     /* 起步电流（静摩擦补偿）：先 0，标定后再填 */
/* ---- 速控：摇杆 → 目标角速度 ---- */
#define GIMBAL_YAW_SPEED_MAX   3.0f     /* 满杆目标角速度 = 400 °/s ÷ 57.3 */
#define GIMBAL_YAW_OMEGA_FF_EN 0U       /*云台YAW速度前馈开关*/
/* ---- 位控：角度外环 ---- */
#define GIMBAL_YAW_KP_ANG      39.0f     /* 单位 (rad/s)/rad；反馈 = 陀螺积分角 yaw_imu_lpf */
#define GIMBAL_YAW_KI_ANG      0.0f      /* 先纯 P；位控的稳态误差用速度前馈消，不是靠 I（中科大 6.5）*/
#define GIMBAL_YAW_W_REF_LIM   20.0f     /* 期望角速度限幅 (rad/s) = 1146 °/s；防大误差时 ω_ref 爆掉 */
#define GIMBAL_YAW_INT_LIM     0.0f      /* KI_ANG=0，用不上 */
#define GIMBAL_YAW_TGT_RATE    0.0038f   /* 位控：摇杆积分成目标角度的速率（满杆 3.8 rad/s ≈ 218 °/s）*/
#define GIMBAL_YAW_TGT_LIMIT   10000.0f/* ★软限位；给这么大 ≈ 没限位 */

/* ---- 输出限幅（0x1FF 电压帧手册量程 ±25000）---- */
#define GIMBAL_YAW_OUT_LIM     20000.0f

/* ---- Watch 观测量 ---- */
volatile float   yaw_tgt       = 0.0f;  /* 位控：目标角度（连续多圈）*/
volatile float   yaw_err       = 0.0f;  /* 位控：角度误差（计数）*/
volatile float   yaw_omega_ref = 0.0f;  /* ★期望角速度 (rad/s)（两层共用的中间量）*/
volatile float   yaw_omega_ff  = 0.0f;  /* ★速度前馈项 (rad/s)（只有位控用）*/
volatile float   yaw_omega_fb  = 0.0f;  /* ★实际角速度 (rad/s)（陀螺，已去零偏）*/
volatile float   yaw_out       = 0.0f;  /* ★最终命令值（发出去的就是它）*/
volatile uint8_t yaw_hold      = 0U;    /* 位控：1 = 目标顶在软限位上 */

pid_t   pid_yaw;
static uint8_t yaw_ready = 0U;

static float gimbal_yaw_control(void)
{
    float axis;//摇杆轴归一化值（-1到+1）
    float out;

    /* ① 首次 / 掉线 → 目标贴着当前位置 + PID 复位（原逻辑不变）*/
    if (yaw_ready == 0U)
    {
        pid_init(&pid_yaw, GIMBAL_YAW_KP_ANG, GIMBAL_YAW_KI_ANG, 0.0f,
                 GIMBAL_YAW_W_REF_LIM, GIMBAL_YAW_INT_LIM);
        yaw_tgt       = yaw_imu_lpf;
        yaw_err       = 0.0f;
        yaw_omega_ref = 0.0f;
        yaw_omega_fb  = 0.0f;
        yaw_omega_ff  = 0.0f;
        yaw_out       = 0.0f;
        yaw_hold      = 0U;
        yaw_ready     = 1U;
        return 0.0f;
    }

    if (remote.online == 0U)//遥控器掉线保护
    {
        pid_reset(&pid_yaw);
        yaw_tgt       = yaw_imu_lpf;   /* 掉线期间目标跟着实际，重连不窜 */
        yaw_err       = 0.0f;
        yaw_omega_ref = 0.0f;
        yaw_omega_fb  = 0.0f;
        yaw_omega_ff  = 0.0f;
        yaw_out       = 0.0f;
        yaw_hold      = 0U;
        return 0.0f;
    }

    /* ★基准刚换过（上电首次收到反馈 / 掉线恢复）→ 目标也贴到新基准，防跳变暴冲 */
    if (angle_rebased != 0U)
    {
        angle_rebased = 0U;
        yaw_tgt       = yaw_imu_lpf;
        yaw_err       = 0.0f;
        yaw_omega_ref = 0.0f;
        pid_reset(&pid_yaw);
        return 0.0f;
    }

    /*摇杆（input_axis 已归一化到 ±1，死区内 = 0）*/
    axis = input_axis(INPUT_AXIS_GIMBAL_YAW);

#if (GIMBAL_YAW_SPEED_MODE != 0U)
    /* 速控：摇杆直接给目标角速度；不积分角度 ⇒ 没有"冲过头"这回事 */
    yaw_omega_ref = axis * GIMBAL_YAW_SPEED_MAX;
    yaw_omega_ff  = 0.0f;              /* ★速控不用前馈 */
    yaw_hold      = 0U;
    yaw_err       = 0.0f;              /* 速控下没有"角度误差"这个概念 */
#else
    /*位控*/
    yaw_tgt += axis * GIMBAL_YAW_TGT_RATE;
    yaw_hold = 0U;
    if (yaw_tgt >  GIMBAL_YAW_TGT_LIMIT) { yaw_tgt =  GIMBAL_YAW_TGT_LIMIT; yaw_hold = 1U; }
    if (yaw_tgt < -GIMBAL_YAW_TGT_LIMIT) { yaw_tgt = -GIMBAL_YAW_TGT_LIMIT; yaw_hold = 1U; }
    yaw_err = gimbal_wrap_pi(yaw_tgt - yaw_imu_lpf);

#if (GIMBAL_YAW_OMEGA_FF_EN != 0U)
    yaw_omega_ff = axis * GIMBAL_YAW_SPEED_MAX;      /* ★前馈 = 摇杆直接给的角速度 */
#else
    yaw_omega_ff = 0.0f;
#endif

    yaw_omega_ref = pid_calc(&pid_yaw, yaw_imu_lpf + yaw_err, yaw_imu_lpf)+ yaw_omega_ff;
    /*pid_calc实际上传入值应该是target和actual。然后target-actuall得到error，但是上方已经算出来了error，因此这里只能用error表示target*/
    /* ω_ref总限幅*/
    if (yaw_omega_ref >  (float)GIMBAL_YAW_W_REF_LIM) { yaw_omega_ref =  (float)GIMBAL_YAW_W_REF_LIM; }
    if (yaw_omega_ref < -(float)GIMBAL_YAW_W_REF_LIM) { yaw_omega_ref = -(float)GIMBAL_YAW_W_REF_LIM; }
#endif

    /* ④ 速度环：KP_W × (期望角速度 − 实际角速度)
       反馈 = 投影到重力方向后的真 yaw 角速度（已减掉上电零偏）
       ⚠️ 陀螺掉线 ⇒ 速度环没有反馈 ⇒ 必须停输出（否则拿旧值当恒定偏置一直推云台）*/
    if (imu_online != 0U)
    {
        yaw_omega_fb = imu_gyro_yaw;
        out = GIMBAL_YAW_KP_W * (yaw_omega_ref - yaw_omega_fb);
    }
    else
    {
        yaw_omega_fb = 0.0f;
        out = 0.0f;
    }
    /* ⑤ 起步电流（静摩擦补偿）：按"期望角速度"方向给固定偏置
       依据：中科大 6.6 低配版 T = t·sgn(ω)；ARBATOS 的 GimbalYawKickCurrent 同款 */
    if (yaw_omega_ref > 0.0f)      { out += GIMBAL_YAW_KICK; }
    else if (yaw_omega_ref < 0.0f) { out -= GIMBAL_YAW_KICK; }

    /* ⑥ 限幅（pid_calc 只夹了外环输出，叠加起步电流后必须再夹一次）*/
    if (out >  (float)GIMBAL_YAW_OUT_LIM) { out =  (float)GIMBAL_YAW_OUT_LIM; }
    if (out < -(float)GIMBAL_YAW_OUT_LIM) { out = -(float)GIMBAL_YAW_OUT_LIM; }

    yaw_out = out;
    return out;
}

/* ============ 步5d-2：PITCH 角度环（闭环）============
   基准 = 固定的"水平值"：上电时云台在任意位置，都会自己往水平靠。
   ⚠️ 实测：往下 angle 减小 / 往上 angle 增大 ⇒ 不取反（正命令 = 往上） */

#define GIMBAL_PITCH_LEVEL      6000    /* ★实测：云台水平时 motor_gimbal_fb[1].angle 的值 */
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


volatile float   pitch_tgt     = 0.0f;   /* 目标；0 = 水平 */
volatile float   pitch_fb      = 0.0f;   /* 反馈；正 = 水平上方 */
volatile float   pitch_err     = 0.0f;   /* 误差 = 目标 − 反馈 */
volatile float   pitch_pid_out = 0.0f;   /* PID 输出（= 要发给电调的电流命令）*/
volatile uint8_t pitch_hold    = 0U;     /* 1 = 目标顶在软限位上 */

pid_t   pid_pitch;
static uint8_t pitch_ready = 0U;


/* 把"原始角 0~8191"换算成"相对水平的偏差"：正 = 水平上方
   为什么要有回绕修正：万一 angle 从 0 跳到 8191，直接相减会得到 ±8192 的假误差 */
static float gimbal_pitch_fb(void)
{
    int32_t d = (int32_t)motor_gimbal_fb[GIMBAL_AXIS_PITCH].angle - (int32_t)GIMBAL_PITCH_LEVEL;

    if (d >  GIMBAL_PITCH_ENC_HALF) { d -= GIMBAL_PITCH_ENC_RANGE; }
    if (d < -GIMBAL_PITCH_ENC_HALF) { d += GIMBAL_PITCH_ENC_RANGE; }

    return (float)d;
}

/* 步5d-3：只做"松手自稳" —— 目标固定为水平，不接摇杆 */
static float gimbal_pitch_control(void)
{
    /* ① 首次上电 → PID 复位，本帧先不发电流 */
    if (pitch_ready == 0U)
    {
        pid_init(&pid_pitch, GIMBAL_PITCH_KP, GIMBAL_PITCH_KI, GIMBAL_PITCH_KD,
                 GIMBAL_PITCH_OUT_LIM, GIMBAL_PITCH_INT_LIM);
        pitch_tgt     = 0.0f;
        pitch_fb      = gimbal_pitch_fb();
        pitch_err     = 0.0f;
        pitch_pid_out = 0.0f;
        pitch_ready   = 1U;
        return 0.0f;
    }

    /* ② 目标恒 = 0 = 水平；反馈以"水平 = 0、上方为正"为单位 → 直接相减，不取反 */
    pitch_tgt     = 0.0f;
    pitch_fb      = gimbal_pitch_fb();
    pitch_err     = pitch_tgt - pitch_fb;
    pitch_pid_out = pid_calc(&pid_pitch, pitch_tgt, pitch_fb) * GIMBAL_PITCH_DIR;

    return pitch_pid_out;
}


void gimbal_run(void)
{
    float out;

    gimbal_angle_track();               /* 把6020的0-8191机械角度累加成连续多圈角*/
    yaw_angle_deg = (float)yaw_angle_acc * (360.0f / 8192.0f);   /* 计数 → 度 */
    gimbal_imu_angle_update();          /* 从 IMU 读角度数据，作为角度环的反馈。*/
    yaw_imu_deg   = yaw_imu_lpf * 57.29578f;                     /* rad → 度 */
    /* ========== YAW（6020 @ CAN1）：角度环 ========== */
#if (GIMBAL_RAW_TEST_ENABLE != 0U)
    out = gimbal_yaw_control();//算YAW的命令值
#else
    out = 0.0f;
#endif
    motor_gimbal_send_yaw((int16_t)out);     /* 把YAW算出来的命令值发出去 */

    /* ========== PITCH（6623 @ CAN2）：角度环（闭环）========== */
#if (GIMBAL_RAW_TEST_ENABLE != 0U)
    out = gimbal_pitch_control();
#else
    out = 0.0f;
#endif
    motor_gimbal_send_pitch((int16_t)out);//同上
}