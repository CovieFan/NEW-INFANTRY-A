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

volatile int32_t yaw_angle_acc   = 0;   /* 编码器累计计数（带回绕修正）。注意：反馈断开 >50ms 会清零，只能当"上次对齐以来的增量"，不是绝对角度 */
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
    int16_t  raw = motor_gimbal[GIMBAL_AXIS_YAW].fb.angle;
    int32_t  d;
    uint32_t now = HAL_GetTick();
    uint32_t last_rx = motor_gimbal[GIMBAL_AXIS_YAW].fb.last_ms;  

    if (last_rx == 0U)
    {
        return;
    }

    if ((angle_started == 0U) || (last_rx == 0U) ||
        ((now - last_rx) > GIMBAL_ANGLE_GAP))
    {
        yaw_angle_raw   = raw;
        yaw_angle_acc   = 0;
        yaw_angle_delta = 0;
        angle_started   = 1U;
        angle_rebased   = 1U;     
        return;
    }

    d = (int32_t)raw - (int32_t)yaw_angle_raw;
    if (d >  GIMBAL_ENC_HALF) { d -= GIMBAL_ENC_RANGE; }
    if (d < -GIMBAL_ENC_HALF) { d += GIMBAL_ENC_RANGE; }

    yaw_angle_delta  = (int16_t)d;
    yaw_angle_raw    = raw;
    yaw_angle_acc   += d;          //得到编码器计数累加值
}

#define GIMBAL_IMU_DT        0.001f//积分步长，是由TIM6 1KHz决定的，每秒一千次
#define GIMBAL_IMU_LPF_ALPHA 0.377f//一阶低通滤波系数alpha

static void gimbal_imu_angle_update(void)//陀螺仪角度积分
{

    if (imu_online == 0U){return;}

    yaw_imu_angle += imu_gyro_yaw * GIMBAL_IMU_DT;//陀螺仪角度原始积分
    // if (imu_ang_inited == 0U)     //首次：低通直接对齐，不从 0 慢慢爬 
    // {
    //     yaw_imu_lpf    = yaw_imu_angle;
    //     imu_ang_inited = 1U;
    //     return;
    // }
    yaw_imu_lpf += GIMBAL_IMU_LPF_ALPHA * (yaw_imu_angle - yaw_imu_lpf);//一阶低通滤波后得到的IMU角度
}

//回绕处理
#define GIMBAL_PI 3.14159265358979f
static float gimbal_wrap_pi(float ang)//保障角度误差在正负pai之内
{
    while (ang >  GIMBAL_PI) { ang -= 2.0f * GIMBAL_PI; }
    while (ang < -GIMBAL_PI) { ang += 2.0f * GIMBAL_PI; }
    return ang;
}

volatile gimbal_cfg_t gimbal_cfg =
{
    .enable = 1U,
    .yaw =
    {
        .speed_mode  = 0U,        /* 1 = 速控；0 = 位控 */
        .omega_ff_en = 1U,        /* 位控：速度前馈开关 */
        .kp_w        = 5000.0f,    /* 速度环 P：命令值 / (rad/s) */
        .kick        = 0.0f,      /* 起步电流（静摩擦补偿） */
        .speed_max   = 4.0f,      /* 速控：满杆目标角速度 (rad/s) */
        .kp_ang      = 5.0f,     /* 位控角度环 P */
        .ki_ang      = 0.0f,      /* 稳态误差靠速度前馈消，不靠 I */
        .w_ref_lim   = 20.0f,     /* 期望角速度限幅 (rad/s) */
        .int_lim     = 0.0f,      /* ki_ang = 0，用不上 */
        .tgt_rate    = 0.0038f,   /* 位控：摇杆积分速率 (rad/tick) */
        .out_lim     = 24000.0f,  /* 输出限幅（0x1FF 量程 +-25000） */
    },
    .pitch =
    {
        .level       = 6000,      /* 云台水平时的编码器值（实测） */
        .kp          = 12.0f,
        .ki          = 0.01f,
        .kd          = 0.0f,
        .out_lim     = 2000.0f,   /* 必须 <= motor_gimbal_cfg[1].out_max */
        .int_lim     = 200000.0f,
        .dir         = -1.0f,
    },
};


volatile float   yaw_tgt       = 0.0f;  /* 位控：目标角度（连续多圈）*/
volatile float   yaw_err       = 0.0f;  /* 位控：角度误差（计数）*/
volatile float   yaw_omega_ref = 0.0f;  /* ★期望角速度 (rad/s)（两层共用的中间量）*/
volatile float   yaw_omega_ff  = 0.0f;  /* ★速度前馈项 (rad/s)（只有位控用）*/
volatile float   yaw_omega_fb  = 0.0f;  /* ★实际角速度 (rad/s)（陀螺，已去零偏）*/
volatile float   yaw_out       = 0.0f;  /* ★最终命令值（发出去的就是它）*/

pid_t   pid_yaw;
static uint8_t yaw_ready = 0U;

/* 每 tick 把配置值同步进 PID，但不调 pid_init（那会清积分） */
static void gimbal_pid_sync(pid_t *p, float kp, float ki, float kd,
                            float out_lim, float int_lim)
{
    p->kp             = kp;
    p->ki             = ki;
    p->kd             = kd;
    p->output_limit   = out_lim;
    p->integral_limit = int_lim;
}

static float gimbal_yaw_control(void)
{
    float yaw_stick_norm;//摇杆轴归一化值
    float out;

    if (yaw_ready == 0U)
    {
        pid_init(&pid_yaw, gimbal_cfg.yaw.kp_ang, gimbal_cfg.yaw.ki_ang, 0.0f,
                 gimbal_cfg.yaw.w_ref_lim, gimbal_cfg.yaw.int_lim);
        yaw_tgt       = yaw_imu_lpf;
        yaw_err       = 0.0f;
        yaw_omega_ref = 0.0f;
        yaw_omega_fb  = 0.0f;
        yaw_omega_ff  = 0.0f;
        yaw_out       = 0.0f;
        yaw_ready     = 1U;
        return 0.0f;
    }

    gimbal_pid_sync(&pid_yaw, gimbal_cfg.yaw.kp_ang, gimbal_cfg.yaw.ki_ang, 0.0f,
                    gimbal_cfg.yaw.w_ref_lim, gimbal_cfg.yaw.int_lim);

    if (remote.online == 0U)
    {
        pid_reset(&pid_yaw);
        yaw_tgt       = yaw_imu_lpf; //遥控器掉线期间保持自稳  
        yaw_err       = 0.0f;
        yaw_omega_ref = 0.0f;
        yaw_omega_fb  = 0.0f;
        yaw_omega_ff  = 0.0f;
        yaw_out       = 0.0f;
        return 0.0f;
    }

    /* ★基准刚换过（上电首次收到反馈 / 掉线恢复）→ 目标也贴到新基准，防跳变暴冲 */
    if (angle_rebased != 0U)
    {
        angle_rebased = 0U;
        yaw_tgt       = yaw_imu_lpf;//同样保持自稳
        yaw_err       = 0.0f;
        yaw_omega_ref = 0.0f;
        pid_reset(&pid_yaw);
        return 0.0f;
    }

    yaw_stick_norm = input_axis(INPUT_AXIS_GIMBAL_YAW);

    if (gimbal_cfg.yaw.speed_mode != 0U)
    {
        //速控
        yaw_omega_ref = yaw_stick_norm * gimbal_cfg.yaw.speed_max;
        yaw_omega_ff  = 0.0f;
        yaw_err       = 0.0f;
    }
    else
    {
        //位控
        yaw_tgt += yaw_stick_norm * gimbal_cfg.yaw.tgt_rate;
        yaw_err = gimbal_wrap_pi(yaw_tgt - yaw_imu_lpf);//保证误差在正负pi内，可成为归pi化

        if (gimbal_cfg.yaw.omega_ff_en != 0U)//前馈开关
        {
            yaw_omega_ff = yaw_stick_norm * gimbal_cfg.yaw.speed_max;   /* 前馈 = 摇杆直接给的角速度 */
        }
        else
        {
            yaw_omega_ff = 0.0f;
        }

        /* target = fb + err，error 就是上面算好的 yaw_err */
        yaw_omega_ref = pid_calc(&pid_yaw, yaw_imu_lpf + yaw_err, yaw_imu_lpf) + yaw_omega_ff;//设定值ref就是pid算出来的值加上前馈
        if (yaw_omega_ref >  gimbal_cfg.yaw.w_ref_lim) { yaw_omega_ref =  gimbal_cfg.yaw.w_ref_lim; }
        if (yaw_omega_ref < -gimbal_cfg.yaw.w_ref_lim) { yaw_omega_ref = -gimbal_cfg.yaw.w_ref_lim; }
    }

    if (imu_online != 0U)
    {
        yaw_omega_fb = imu_gyro_yaw;
        out = gimbal_cfg.yaw.kp_w * (yaw_omega_ref - yaw_omega_fb);
    }
    else
    {
        yaw_omega_fb = 0.0f;
        out = 0.0f;
    }

    if (yaw_omega_ref > 0.0f)      { out += gimbal_cfg.yaw.kick; }
    else if (yaw_omega_ref < 0.0f) { out -= gimbal_cfg.yaw.kick; }

    if (out >  gimbal_cfg.yaw.out_lim) { out =  gimbal_cfg.yaw.out_lim; }//限幅削平输出
    if (out < -gimbal_cfg.yaw.out_lim) { out = -gimbal_cfg.yaw.out_lim; }

    yaw_out = out;
    return out;
}

//PITCH 角度环闭环
#define GIMBAL_PITCH_ENC_RANGE  8192L   /* 13 位编码器 */
#define GIMBAL_PITCH_ENC_HALF   4096L   /* 半圈，用于回绕修正 */

volatile float   pitch_tgt     = 0.0f;   /* 目标；0 = 水平 */
volatile float   pitch_fb      = 0.0f;   /* 反馈；正 = 水平上方 */
volatile float   pitch_err     = 0.0f;   /* 误差 = 目标 − 反馈 */
volatile float   pitch_pid_out = 0.0f;   /* PID 输出（= 要发给电调的电流命令）*/

pid_t   pid_pitch;
static uint8_t pitch_ready = 0U;

/* 把"原始角 0~8191"换算成"相对水平的偏差"：正 = 水平上方
   为什么要有回绕修正：万一 angle 从 0 跳到 8191，直接相减会得到 ±8192 的假误差 */
static float gimbal_pitch_fb(void)
{
    int32_t d = (int32_t)motor_gimbal[GIMBAL_AXIS_PITCH].fb.angle - (int32_t)gimbal_cfg.pitch.level;
    if (d >  GIMBAL_PITCH_ENC_HALF) { d -= GIMBAL_PITCH_ENC_RANGE; }
    if (d < -GIMBAL_PITCH_ENC_HALF) { d += GIMBAL_PITCH_ENC_RANGE; }

    return (float)d;
}

static float gimbal_pitch_control(void)
{
    if (pitch_ready == 0U)
    {
        pid_init(&pid_pitch, gimbal_cfg.pitch.kp, gimbal_cfg.pitch.ki, gimbal_cfg.pitch.kd,
                 gimbal_cfg.pitch.out_lim, gimbal_cfg.pitch.int_lim);
        pitch_tgt     = 0.0f;
        pitch_fb      = gimbal_pitch_fb();
        pitch_err     = 0.0f;
        pitch_pid_out = 0.0f;
        pitch_ready   = 1U;
        return 0.0f;
    }

    gimbal_pid_sync(&pid_pitch, gimbal_cfg.pitch.kp, gimbal_cfg.pitch.ki, gimbal_cfg.pitch.kd,
                    gimbal_cfg.pitch.out_lim, gimbal_cfg.pitch.int_lim);

    pitch_tgt     = 0.0f;
    pitch_fb      = gimbal_pitch_fb();
    pitch_err     = pitch_tgt - pitch_fb;
    pitch_pid_out = pid_calc(&pid_pitch, pitch_tgt, pitch_fb) * gimbal_cfg.pitch.dir;

    return pitch_pid_out;
}

void gimbal_run(void)
{
    float out;

    gimbal_angle_track();              
    yaw_angle_deg = (float)yaw_angle_acc * (360.0f / 8192.0f);   //算一下YAW电机相对底盘转了多少度

    gimbal_imu_angle_update();          
    yaw_imu_deg   = yaw_imu_lpf * 57.29578f; //IMU输出的是角速度，角速度积分得到的角度是弧度制，因此这里需要手动转化为角度                    

    out = (gimbal_cfg.enable != 0U) ? gimbal_yaw_control() : 0.0f;
    motor_gimbal_send_yaw((int16_t)out);     

    out = (gimbal_cfg.enable != 0U) ? gimbal_pitch_control() : 0.0f;
    motor_gimbal_send_pitch((int16_t)out);//同上
}
