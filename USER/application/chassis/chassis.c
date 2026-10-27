/* chassis.c —— 底盘应用层：摇杆 → 坐标转换 → 四轮目标 → 电流 */
#include "chassis.h"
#include "motor.h"
#include "main.h"                      
#include "remote.h"
#include "pid.h"
#include "input.h"
#include "gimbal.h"
#include <math.h>

#define CHASSIS_YAW_ENC_RANGE 8192L
#define CHASSIS_YAW_ENC_HALF  4096L
#define CHASSIS_TWO_PI        6.28318530718f
/* 宽松数值保护，不是电机能力或PID稳定边界。 */
#define CHASSIS_COMMAND_MAX   ((float)INT16_MAX)
#define CHASSIS_GAIN_MAX      1000000.0f
#define CHASSIS_INTEGRAL_MAX  1000000000.0f
#define CHASSIS_PID_ERROR_MAX (2.0f * (CHASSIS_COMMAND_MAX + 1.0f))
/* 下标 0~3 = 0x201 FR / 0x202 FL / 0x203 RL / 0x204 RR（实测） */
/* +1 = 正电流对应前进（左右镜像安装） */
const int8_t motor_dir[4] = { +1, -1, -1, +1 };

volatile int16_t dbg_cur[4] = { 0, 0, 0, 0 };
volatile chassis_mode_e chassis_mode = CHASSIS_MODE_PID_TEST;
volatile chassis_control_fault_e chassis_control_fault = CHASSIS_CONTROL_FAULT_NONE;
volatile chassis_cfg_t chassis_cfg =
{
    .fwd_cur = 2500, .strafe_cur = 2500, .spin_cur = 2500, .cur_max = 4500,
    .spin_fixed_rpm = 2000.0f,
    .follow_en = 1U,
    .follow_kp = 3000.0f,
    .follow_max_rpm = 2000.0f,
    .follow_deadband_rad = 0.052359878f, /* 3 * pi / 180 = 0.05236 rad */
    .max_rpm = 5000.0f, .kp = 15.0f, .ki = 0.0f, .kd = 0.0f,
    .out_lim = 3000.0f, .int_lim = 3000.0f,
    .move_frame = CHASSIS_FRAME_GIMBAL,
    .yaw_zero_ecd = 5500,     /* 实测：枪口与底盘正前方对齐时的编码器值 */
    .yaw_ecd_dir = +1,       /* 实测：枪口左转时编码器增加；直驱 */
    .yaw_timeout_ms = 100U,
};

volatile float   dbg_vx  = 0.0f;
volatile float   dbg_vy  = 0.0f;
volatile float   dbg_wz  = 0.0f;
volatile float   dbg_vx_input = 0.0f;
volatile float   dbg_vy_input = 0.0f;

volatile float    chassis_yaw_rel_rad = 0.0f;
volatile uint32_t chassis_yaw_age_ms = UINT32_MAX;
volatile uint8_t  chassis_yaw_valid = 0U;
volatile uint8_t  chassis_wait_neutral = 1U;
volatile uint8_t  chassis_follow_active = 0U;
volatile float    chassis_translation_scale = 0.0f;
volatile float    chassis_wz_applied = 0.0f;

volatile float   pid_tar[4] = { 0, 0, 0, 0 };   /* Watch：四轮目标（前进为正） */
volatile float   pid_fb [4] = { 0, 0, 0, 0 };   /* Watch：四轮反馈（前进为正） */
volatile float   pid_scale = 1.0f;
volatile uint8_t pid_ok = 0U;

static pid_t chassis_pid[4];

static uint8_t chassis_range_valid(float value, float low, float high)
{
    return (uint8_t)(isfinite(value) && (value >= low) && (value <= high));
}

static uint8_t chassis_pid_config_valid(const chassis_cfg_t *cfg)
{
    return (uint8_t)((cfg->follow_en <= 1U) &&
        chassis_range_valid(cfg->max_rpm, 0.0f, CHASSIS_COMMAND_MAX) && (cfg->max_rpm > 0.0f) &&
        chassis_range_valid(cfg->spin_fixed_rpm, -CHASSIS_COMMAND_MAX, CHASSIS_COMMAND_MAX) &&
        chassis_range_valid(cfg->kp, 0.0f, CHASSIS_GAIN_MAX) &&
        chassis_range_valid(cfg->ki, 0.0f, CHASSIS_GAIN_MAX) &&
        chassis_range_valid(cfg->kd, 0.0f, CHASSIS_GAIN_MAX) &&
        chassis_range_valid(cfg->out_lim, 0.0f, CHASSIS_COMMAND_MAX) && (cfg->out_lim > 0.0f) &&
        chassis_range_valid(cfg->int_lim, 0.0f, CHASSIS_INTEGRAL_MAX));
}

/* STOP、失效和等待归中都走这里，主动发零，避免保留上一次电流。 */
static void chassis_stop(void)
{
    static const int16_t zero[4] = { 0, 0, 0, 0 };
    uint8_t i;

    chassis_wait_neutral = 1U;
    chassis_follow_active = 0U;
    chassis_translation_scale = 0.0f;
    chassis_wz_applied = 0.0f;
    dbg_vx_input = 0.0f; dbg_vy_input = 0.0f;
    dbg_vx = 0.0f; dbg_vy = 0.0f; dbg_wz = 0.0f;
    pid_scale = 0.0f;
    for (i = 0U; i < 4U; i++)
    {
        pid_reset(&chassis_pid[i]);
        pid_tar[i] = 0.0f;
        pid_fb[i] = (float)motor_chassis[i].fb.speed_rpm * (float)motor_dir[i];
        dbg_cur[i] = 0;
    }
    motor_chassis_send_current(zero);
}

static void chassis_fail(chassis_control_fault_e fault)
{
    chassis_control_fault = fault;
    chassis_stop();
}

static void chassis_yaw_update(const chassis_cfg_t *cfg)
{
    int16_t raw = motor_gimbal[GIMBAL_AXIS_YAW].fb.angle;
    uint32_t last_ms = motor_gimbal[GIMBAL_AXIS_YAW].fb.last_ms;
    uint32_t now = HAL_GetTick();
    int32_t d;

    /* 各读一次作局部采样。接收端先写 angle 后写时间戳；被 TIM6 打断时
       可能是新角度配旧时间戳，年龄判断偏保守，不宣称严格同帧快照。 */
    chassis_yaw_age_ms = (last_ms != 0U) ? (now - last_ms) : UINT32_MAX;
    chassis_yaw_valid = 0U;
    chassis_yaw_rel_rad = 0.0f;
    if ((raw < 0) || (raw >= CHASSIS_YAW_ENC_RANGE) ||
        (cfg->yaw_zero_ecd < 0) || (cfg->yaw_zero_ecd >= CHASSIS_YAW_ENC_RANGE) ||
        ((cfg->yaw_ecd_dir != 1) && (cfg->yaw_ecd_dir != -1)) ||
        (last_ms == 0U) || (cfg->yaw_timeout_ms == 0U) ||
        (chassis_yaw_age_ms > cfg->yaw_timeout_ms))
    {
        return;
    }

    d = (int32_t)raw - (int32_t)cfg->yaw_zero_ecd;
    if (d >  CHASSIS_YAW_ENC_HALF) { d -= CHASSIS_YAW_ENC_RANGE; }
    if (d < -CHASSIS_YAW_ENC_HALF) { d += CHASSIS_YAW_ENC_RANGE; }
    chassis_yaw_rel_rad = (float)cfg->yaw_ecd_dir * (float)d *
                          (CHASSIS_TWO_PI / (float)CHASSIS_YAW_ENC_RANGE);
    chassis_yaw_valid = 1U;
}

static void chassis_rc_run(const chassis_cfg_t *cfg)
{
    float   vx;
    float   vy;
    float   wz;
    float   wf[4];
    int16_t cur[4] = { 0, 0, 0, 0 };
    uint8_t i;

    if (cfg->cur_max < 0)
    {
        chassis_fail(CHASSIS_CONTROL_FAULT_CONFIG);
        return;
    }

    /* 掉线 -> 立即全 0 电流 */
    if (remote.online == 0U)
    {
        chassis_stop();
        return;
    }

    vx = input_axis(INPUT_AXIS_CHASSIS_X);
    vy = input_axis(INPUT_AXIS_CHASSIS_Y);
    wz = input_axis(INPUT_AXIS_CHASSIS_WZ);
    if (chassis_wait_neutral != 0U)
    {
        chassis_stop();
        if ((vx == 0.0f) && (vy == 0.0f) && (wz == 0.0f))
        {
            chassis_wait_neutral = 0U;
        }
        return;
    }

    vx *= (float)cfg->fwd_cur;
    vy *= (float)cfg->strafe_cur;
    wz *= (float)cfg->spin_cur;
    dbg_vx_input = vx; dbg_vy_input = vy;

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

        if (!isfinite(c)) { chassis_fail(CHASSIS_CONTROL_FAULT_NUMERIC); return; }
        if (c >  (float)cfg->cur_max) { c =  (float)cfg->cur_max; }
        if (c < -(float)cfg->cur_max) { c = -(float)cfg->cur_max; }

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

static void chassis_pid_run(const chassis_cfg_t *cfg)
{
    float   x_norm;
    float   y_norm;
    float   vx_input;
    float   vy_input;
    float   vx;
    float   vy;
    float   wz;
    float   tgt[4];
    int16_t cur[4] = { 0, 0, 0, 0 };
    float   maxabs = 0.0f;
    float   scale  = 1.0f;
    float   move_scale = 1.0f;
    uint8_t sw2 = remote.sw2;
    uint8_t spin_on = (sw2 == INPUT_SW_MID) ? 1U : 0U;
    uint8_t follow_on = (uint8_t)((spin_on == 0U) && (cfg->follow_en != 0U) &&
                                (cfg->move_frame == CHASSIS_FRAME_GIMBAL));
    uint8_t i;

    /* 校验先于PID初始化、同步和目标计算。 */
    if (chassis_pid_config_valid(cfg) == 0U)
    {
        chassis_fail(CHASSIS_CONTROL_FAULT_CONFIG);
        return;
    }

    if (pid_ok == 0U)
    {
        for (i = 0U; i < 4U; i++)
        {
            pid_init(&chassis_pid[i], cfg->kp, cfg->ki, cfg->kd,
                     cfg->out_lim, cfg->int_lim);
        }
        pid_ok = 1U;
    }

    for (i = 0U; i < 4U; i++)
    {
        chassis_pid[i].kp             = cfg->kp;
        chassis_pid[i].ki             = cfg->ki;
        chassis_pid[i].kd             = cfg->kd;
        chassis_pid[i].output_limit   = cfg->out_lim;
        chassis_pid[i].integral_limit = cfg->int_lim;
    }

    chassis_yaw_update(cfg);
    if ((remote.online == 0U) ||
        ((sw2 != INPUT_SW_UP) && (sw2 != INPUT_SW_MID) && (sw2 != INPUT_SW_DOWN)) ||
        ((cfg->move_frame != CHASSIS_FRAME_BODY) && (cfg->move_frame != CHASSIS_FRAME_GIMBAL)) ||
        ((cfg->move_frame == CHASSIS_FRAME_GIMBAL) && (chassis_yaw_valid == 0U)))
    {
        chassis_stop();
        return;
    }

    x_norm = input_axis(INPUT_AXIS_CHASSIS_X);
    y_norm = input_axis(INPUT_AXIS_CHASSIS_Y);
    if ((follow_on != 0U) &&
        (!chassis_range_valid(cfg->follow_kp, 0.0f, CHASSIS_GAIN_MAX) || (cfg->follow_kp <= 0.0f) ||
         !chassis_range_valid(cfg->follow_max_rpm, 0.0f, CHASSIS_COMMAND_MAX) || (cfg->follow_max_rpm <= 0.0f) ||
         !isfinite(cfg->follow_deadband_rad) || (cfg->follow_deadband_rad < 0.0f) ||
         (cfg->follow_deadband_rad >= (CHASSIS_TWO_PI * 0.5f))))
    {
        chassis_fail(CHASSIS_CONTROL_FAULT_CONFIG);
        return;
    }
    if (chassis_wait_neutral != 0U)
    {
        chassis_stop();
        if ((x_norm == 0.0f) && (y_norm == 0.0f) && (spin_on == 0U))
        {
            chassis_wait_neutral = 0U;
        }
        return;
    }

    /* 输入参考系中的平移指令，沿用现有转子 rpm 尺度。 */
    vx_input = x_norm * cfg->max_rpm;
    vy_input = y_norm * cfg->max_rpm;
    dbg_vx_input = vx_input; dbg_vy_input = vy_input;
    vx = vx_input;
    vy = vy_input;
    if (cfg->move_frame == CHASSIS_FRAME_GIMBAL)
    {
        float yaw_rel_rad = chassis_yaw_rel_rad;
        float sin_yaw = sinf(yaw_rel_rad);
        float cos_yaw = cosf(yaw_rel_rad);

        /* 相对角左转为正；按现有底盘侧向约定用负角变换，避免左右镜像。 */
        vx =  cos_yaw * vx_input + sin_yaw * vy_input;
        vy = -sin_yaw * vx_input + cos_yaw * vy_input;
    }
    /* 小陀螺优先；其余时候按实际角差跟随。
       本车实测正 wz 使左正的相对角增大；跟随取负号，才会向零角差收敛。 */
    wz = 0.0f;
    if (spin_on != 0U)
    {
        wz = cfg->spin_fixed_rpm;
    }
    else if ((follow_on != 0U) &&
             (fabsf(chassis_yaw_rel_rad) > cfg->follow_deadband_rad))
    {
        wz = -cfg->follow_kp * chassis_yaw_rel_rad;
        if (wz >  cfg->follow_max_rpm) { wz =  cfg->follow_max_rpm; }
        if (wz < -cfg->follow_max_rpm) { wz = -cfg->follow_max_rpm; }
        chassis_follow_active = 1U;
    }
    dbg_vx = vx; dbg_vy = vy; dbg_wz = wz;

    if (spin_on != 0U)
    {
        float move_rpm = fabsf(vx) + fabsf(vy);
        float available_rpm = cfg->max_rpm - fabsf(wz);

        /* 自转本身已超轮速能力，不能假装仍能保持该目标。 */
        if (available_rpm < 0.0f) { chassis_fail(CHASSIS_CONTROL_FAULT_CONFIG); return; }
        if (!isfinite(move_rpm)) { chassis_fail(CHASSIS_CONTROL_FAULT_NUMERIC); return; }
        /* 先给自转预留轮速，余量只缩平移，保持驾驶方向。 */
        if (move_rpm > available_rpm)
        {
            move_scale = available_rpm / move_rpm;
            vx *= move_scale;
            vy *= move_scale;
        }
    }

    /* 逆运动学 -> 四轮目标（0~3 = FR/FL/RL/RR） */
    tgt[0] = vx + vy + wz;    /* FR */
    tgt[1] = vx - vy - wz;    /* FL */
    tgt[2] = vx + vy - wz;    /* RL */
    tgt[3] = vx - vy + wz;    /* RR */

    /* 普通驾驶仍等比缩放；小陀螺已预留旋转量，不能再一起缩小。 */
    for (i = 0U; i < 4U; i++)
    {
        if (!isfinite(tgt[i])) { chassis_fail(CHASSIS_CONTROL_FAULT_NUMERIC); return; }
        float a = (tgt[i] < 0.0f) ? -tgt[i] : tgt[i];
        if (a > maxabs) { maxabs = a; }
    }
    if (maxabs > cfg->max_rpm)
    {
        if (spin_on == 0U)
        {
            scale = cfg->max_rpm / maxabs;
            for (i = 0U; i < 4U; i++) { tgt[i] *= scale; }
        }
        else
        {
            /* 前面的分配已满足上限，此处仅收紧浮点舍入产生的边界越界。 */
            for (i = 0U; i < 4U; i++)
            {
                if (tgt[i] > cfg->max_rpm) { tgt[i] = cfg->max_rpm; }
                if (tgt[i] < -cfg->max_rpm) { tgt[i] = -cfg->max_rpm; }
            }
        }
    }
    pid_scale = scale;
    chassis_translation_scale = move_scale * scale;
    chassis_wz_applied = (tgt[0] - tgt[1] - tgt[2] + tgt[3]) * 0.25f;

    for (i = 0U; i < 4U; i++)
    {
        float fb = (float)motor_chassis[i].fb.speed_rpm * (float)motor_dir[i];  /* 前进为正 */
        float command;

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

        if (!chassis_range_valid(chassis_pid[i].integral, -CHASSIS_INTEGRAL_MAX, CHASSIS_INTEGRAL_MAX) ||
            !chassis_range_valid(chassis_pid[i].last_error, -CHASSIS_PID_ERROR_MAX, CHASSIS_PID_ERROR_MAX))
        {
            chassis_fail(CHASSIS_CONTROL_FAULT_NUMERIC);
            return;
        }
        command = pid_calc(&chassis_pid[i], tgt[i], fb) * (float)motor_dir[i];
        if (!isfinite(command)) { chassis_fail(CHASSIS_CONTROL_FAULT_NUMERIC); return; }
        if (command >  CHASSIS_COMMAND_MAX) { command =  CHASSIS_COMMAND_MAX; }
        if (command < -CHASSIS_COMMAND_MAX) { command = -CHASSIS_COMMAND_MAX; }
        cur[i] = (int16_t)command;
        dbg_cur[i] = cur[i];
    }

    motor_chassis_send_current(cur);
}


void chassis_run(void)
{
    static chassis_mode_e previous_mode = CHASSIS_MODE_STOP;
    static chassis_frame_e previous_frame = CHASSIS_FRAME_GIMBAL;
    static int16_t previous_zero = -1;
    static int8_t previous_dir = 0;
    static uint8_t previous_follow_en = 0U;
    chassis_mode_e mode = chassis_mode;
    const chassis_cfg_t cfg = chassis_cfg;
    chassis_follow_active = 0U;
    chassis_translation_scale = 0.0f;
    chassis_wz_applied = 0.0f;
    chassis_control_fault = CHASSIS_CONTROL_FAULT_NONE;

    if ((mode != CHASSIS_MODE_STOP) && (mode != CHASSIS_MODE_RC) && (mode != CHASSIS_MODE_PID_TEST))
    {
        chassis_fail(CHASSIS_CONTROL_FAULT_CONFIG);
        return;
    }

    /* WAIT/RUN 暂停底盘；超时已退出自动回正，可经下方有效性检查和归中后运行。
       保留超时告警，不伪装成回正成功；配置故障或未知状态仍停输出。 */
    if ((yaw_home_state != GIMBAL_YAW_HOME_DONE) &&
        !((yaw_home_state == GIMBAL_YAW_HOME_FAULT) &&
          (yaw_home_fault == GIMBAL_YAW_HOME_FAULT_TIMEOUT)))
    {
        chassis_yaw_valid = 0U;
        chassis_yaw_rel_rad = 0.0f;
        chassis_yaw_age_ms = UINT32_MAX;
        chassis_stop();
        return;
    }

    /* 模式或几何配置变化后，先归中；PID 每拍使用这次复制的配置。 */
    if ((mode != previous_mode) ||
        ((mode == CHASSIS_MODE_PID_TEST) &&
         ((cfg.move_frame != previous_frame) ||
          (cfg.yaw_zero_ecd != previous_zero) || (cfg.yaw_ecd_dir != previous_dir) ||
          (cfg.follow_en != previous_follow_en))))
    {
        chassis_wait_neutral = 1U;
    }
    previous_mode = mode;
    previous_frame = cfg.move_frame;
    previous_zero = cfg.yaw_zero_ecd;
    previous_dir = cfg.yaw_ecd_dir;
    previous_follow_en = cfg.follow_en;

    switch (mode)
    {
    case CHASSIS_MODE_RC:
        chassis_yaw_valid = 0U;
        chassis_yaw_rel_rad = 0.0f;
        chassis_yaw_age_ms = UINT32_MAX;
        chassis_rc_run(&cfg);
        break;

    case CHASSIS_MODE_PID_TEST:
        chassis_pid_run(&cfg);
        break;

    case CHASSIS_MODE_STOP:
    default:
        chassis_yaw_valid = 0U;
        chassis_yaw_rel_rad = 0.0f;
        chassis_yaw_age_ms = UINT32_MAX;
        chassis_stop();
        break;
    }
}
