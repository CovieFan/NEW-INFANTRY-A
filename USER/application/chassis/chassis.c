/* chassis.c —— 底盘应用层：摇杆 → 坐标转换 → 四轮目标 → 电流 */
#include "chassis.h"
#include "motor.h"
#include "main.h"                      
#include "remote.h"
#include "pid.h"
#include "input.h"
#include "gimbal.h"
#include "bsp_imu.h"
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

volatile chassis_state_t chassis_state =
{
    /* 模式与故障 */
    .mode          = CHASSIS_MODE_PID_TEST,
    .control_fault = CHASSIS_CONTROL_FAULT_NONE,
    .wait_neutral  = 1U,
    .follow_active = 0U,
    /* 云台相对角 */
    .yaw_rel_rad   = 0.0f,
    .yaw_age_ms    = UINT32_MAX,
    .yaw_valid     = 0U,
    /* 驻车 */
    .park_state     = CHASSIS_PARK_DRIVE,
    .park_fault     = CHASSIS_PARK_FAULT_NONE,
    .park_idle_ms   = 0U,
    .park_brake_ms  = 0U,
    .park_stable_ms = 0U,
    .park_yaw_ecd   = -1,
    .park_yaw_dir   = 0,
    /* 输出诊断 */
    .translation_scale = 0.0f,
    .wz_applied        = 0.0f,
    .pid_tar   = { 0.0f, 0.0f, 0.0f, 0.0f },
    .pid_fb    = { 0.0f, 0.0f, 0.0f, 0.0f },
    .pid_scale = 1.0f,
    .pid_ok    = 0U,
    /* 调试 */
    .dbg_cur      = { 0, 0, 0, 0 },
    .dbg_vx       = 0.0f,
    .dbg_vy       = 0.0f,
    .dbg_wz       = 0.0f,
    .dbg_vx_input = 0.0f,
    .dbg_vy_input = 0.0f,
};
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
    .park =
    {
        .enable = 1U, .idle_ms = 3000U,
        .stop_rpm = 10.0f, .settle_ms = 200U, .brake_timeout_ms = 3000U,
    },
};




static pid_t chassis_pid[4];
static uint32_t park_idle_start_ms;
static uint8_t park_idle_timing = 0U;
static uint32_t park_brake_start_ms;
static uint32_t park_stable_start_ms;
static uint8_t park_settling = 0U;

static void chassis_park_reset(void)
{
    chassis_state.park_state = CHASSIS_PARK_DRIVE;
    chassis_state.park_idle_ms = 0U;
    chassis_state.park_brake_ms = 0U;
    chassis_state.park_stable_ms = 0U;
    chassis_state.park_yaw_ecd = -1;
    chassis_state.park_yaw_dir = 0;
    chassis_state.park_fault = CHASSIS_PARK_FAULT_NONE;
    park_idle_timing = 0U;
    park_settling = 0U;
}

static uint8_t chassis_park_input_active(void)
{
    return (uint8_t)((remote.sw2 == INPUT_SW_MID) ||
        (input_axis(INPUT_AXIS_CHASSIS_X) != 0.0f) || (input_axis(INPUT_AXIS_CHASSIS_Y) != 0.0f) ||
        (input_axis(INPUT_AXIS_GIMBAL_YAW) != 0.0f) || (input_axis(INPUT_AXIS_GIMBAL_PITCH) != 0.0f));
}

static uint8_t chassis_park_wheels_online(uint32_t now)
{
    uint8_t i;
    for (i = 0U; i < 4U; i++)
    {
        uint32_t last_ms = motor_chassis[i].fb.last_ms;
        if ((last_ms == 0U) || ((now - last_ms) > 100U)) { return 0U; }
    }
    return 1U;
}

static uint8_t chassis_park_wheels_stopped(float stop_rpm)
{
    uint8_t i;
    for (i = 0U; i < 4U; i++)
    {
        if (fabsf((float)motor_chassis[i].fb.speed_rpm) > stop_rpm) { return 0U; }
    }
    return 1U;
}

/* 仅决定正常驾驶/驻车；输出仍由现有轮速闭环计算。 */
static void chassis_park_update(const chassis_cfg_t *cfg, uint32_t now)
{
    uint8_t manual_ready = (uint8_t)((gimbal_state.home_state == GIMBAL_YAW_HOME_DONE) ||
        ((gimbal_state.home_state == GIMBAL_YAW_HOME_FAULT) &&
         (gimbal_state.home_fault == GIMBAL_YAW_HOME_FAULT_TIMEOUT) && (gimbal_state.home_manual_ready != 0U)));

    if ((cfg->park.enable == 0U) || (manual_ready == 0U) ||
        (remote.online == 0U) || (remote.last_ms == 0U) || ((now - remote.last_ms) > 150U) ||
        (chassis_state.yaw_valid == 0U) || (chassis_park_input_active() != 0U))
    {
        chassis_park_reset();
        return;
    }
    if (((chassis_state.park_state == CHASSIS_PARK_BRAKE) || (chassis_state.park_state == CHASSIS_PARK_HOLD)) &&
        (chassis_park_wheels_online(now) == 0U))
    {
        chassis_state.park_fault = CHASSIS_PARK_FAULT_WHEEL;
        chassis_state.park_state = CHASSIS_PARK_FAULT;
        return;
    }
    if ((gimbal_cfg.enable != 1U) || (imu_online == 0U) || (gimbal_state.yaw_inhibit != 0U))
    {
        if ((chassis_state.park_state == CHASSIS_PARK_BRAKE) || (chassis_state.park_state == CHASSIS_PARK_HOLD))
        {
            /* YAW短暂失效只撤销角度保持，不能无操作就重新放开底盘跟随。 */
            chassis_state.park_state = CHASSIS_PARK_BRAKE;
            chassis_state.park_yaw_ecd = -1;
            chassis_state.park_yaw_dir = 0;
            park_settling = 0U;
            chassis_state.park_stable_ms = 0U;
            chassis_state.park_brake_ms = 0U;
            park_brake_start_ms = now;
        }
        else { chassis_park_reset(); }
        return;
    }
    if (park_idle_timing == 0U) { park_idle_start_ms = now; park_idle_timing = 1U; }
    chassis_state.park_idle_ms = now - park_idle_start_ms;
    if ((chassis_state.park_state == CHASSIS_PARK_DRIVE) && (chassis_state.park_idle_ms >= cfg->park.idle_ms))
    {
        chassis_state.park_state = CHASSIS_PARK_BRAKE;
        park_brake_start_ms = now;
    }
    if ((chassis_state.park_state != CHASSIS_PARK_DRIVE) && (chassis_park_wheels_online(now) == 0U))
    {
        chassis_state.park_fault = CHASSIS_PARK_FAULT_WHEEL;
        chassis_state.park_state = CHASSIS_PARK_FAULT;
        return;
    }
    if (chassis_state.park_state != CHASSIS_PARK_BRAKE) { return; }
    chassis_state.park_brake_ms = now - park_brake_start_ms;
    if (chassis_park_wheels_stopped(cfg->park.stop_rpm) != 0U)
    {
        if (park_settling == 0U) { park_stable_start_ms = now; park_settling = 1U; }
        chassis_state.park_stable_ms = now - park_stable_start_ms;
        if (chassis_state.park_stable_ms >= cfg->park.settle_ms)
        {
            chassis_state.park_yaw_ecd = motor_gimbal[GIMBAL_AXIS_YAW].fb.angle;
            chassis_state.park_yaw_dir = cfg->yaw_ecd_dir;
            chassis_state.park_state = CHASSIS_PARK_HOLD;
        }
    }
    else { park_settling = 0U; chassis_state.park_stable_ms = 0U; }
    if ((chassis_state.park_state != CHASSIS_PARK_HOLD) &&
        (chassis_state.park_brake_ms >= cfg->park.brake_timeout_ms))
    {
        chassis_state.park_fault = CHASSIS_PARK_FAULT_TIMEOUT;
        chassis_state.park_state = CHASSIS_PARK_FAULT;
    }
}

static uint8_t chassis_range_valid(float value, float low, float high)
{
    return (uint8_t)(isfinite(value) && (value >= low) && (value <= high));
}

static uint8_t chassis_pid_config_valid(const chassis_cfg_t *cfg)
{
    return (uint8_t)((cfg->follow_en <= 1U) && (cfg->park.enable <= 1U) &&
        ((cfg->park.enable == 0U) || ((cfg->park.idle_ms > 0U) && (cfg->park.idle_ms <= UINT32_MAX / 2U) &&
         chassis_range_valid(cfg->park.stop_rpm, 0.0f, CHASSIS_COMMAND_MAX) &&
         (cfg->park.settle_ms > 0U) && (cfg->park.brake_timeout_ms >= cfg->park.settle_ms) &&
         (cfg->park.brake_timeout_ms <= UINT32_MAX / 2U))) &&
        (chassis_state.park_state >= CHASSIS_PARK_DRIVE) && (chassis_state.park_state <= CHASSIS_PARK_FAULT) &&
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

    chassis_state.wait_neutral = 1U;
    chassis_park_reset();
    chassis_state.follow_active = 0U;
    chassis_state.translation_scale = 0.0f;
    chassis_state.wz_applied = 0.0f;
    chassis_state.dbg_vx_input = 0.0f; chassis_state.dbg_vy_input = 0.0f;
    chassis_state.dbg_vx = 0.0f; chassis_state.dbg_vy = 0.0f; chassis_state.dbg_wz = 0.0f;
    chassis_state.pid_scale = 0.0f;
    for (i = 0U; i < 4U; i++)
    {
        pid_reset(&chassis_pid[i]);
        chassis_state.pid_tar[i] = 0.0f;
        chassis_state.pid_fb[i] = (float)motor_chassis[i].fb.speed_rpm * (float)motor_dir[i];
        chassis_state.dbg_cur[i] = 0;
    }
    motor_chassis_send_current(zero);
}

static void chassis_fail(chassis_control_fault_e fault)
{
    chassis_state.control_fault = fault;
    chassis_stop();
}

static void chassis_park_fault_stop(chassis_park_fault_e fault, uint32_t elapsed_ms)
{
    chassis_fail(CHASSIS_CONTROL_FAULT_PARK);
    chassis_state.park_state = CHASSIS_PARK_FAULT; /* 保留故障，不能无操作就重新启动跟随。 */
    chassis_state.park_fault = fault;
    chassis_state.park_brake_ms = elapsed_ms;
}

static void chassis_yaw_update(const chassis_cfg_t *cfg)
{
    int16_t raw = motor_gimbal[GIMBAL_AXIS_YAW].fb.angle;
    uint32_t last_ms = motor_gimbal[GIMBAL_AXIS_YAW].fb.last_ms;
    uint32_t now = HAL_GetTick();
    int32_t d;

    /* 各读一次作局部采样。接收端先写 angle 后写时间戳；被 TIM6 打断时
       可能是新角度配旧时间戳，年龄判断偏保守，不宣称严格同帧快照。 */
    chassis_state.yaw_age_ms = (last_ms != 0U) ? (now - last_ms) : UINT32_MAX;
    chassis_state.yaw_valid = 0U;
    chassis_state.yaw_rel_rad = 0.0f;
    if ((raw < 0) || (raw >= CHASSIS_YAW_ENC_RANGE) ||
        (cfg->yaw_zero_ecd < 0) || (cfg->yaw_zero_ecd >= CHASSIS_YAW_ENC_RANGE) ||
        ((cfg->yaw_ecd_dir != 1) && (cfg->yaw_ecd_dir != -1)) ||
        (last_ms == 0U) || (cfg->yaw_timeout_ms == 0U) ||
        (chassis_state.yaw_age_ms > cfg->yaw_timeout_ms))
    {
        return;
    }

    d = (int32_t)raw - (int32_t)cfg->yaw_zero_ecd;
    if (d >  CHASSIS_YAW_ENC_HALF) { d -= CHASSIS_YAW_ENC_RANGE; }
    if (d < -CHASSIS_YAW_ENC_HALF) { d += CHASSIS_YAW_ENC_RANGE; }
    chassis_state.yaw_rel_rad = (float)cfg->yaw_ecd_dir * (float)d *
                          (CHASSIS_TWO_PI / (float)CHASSIS_YAW_ENC_RANGE);
    chassis_state.yaw_valid = 1U;
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
    if (chassis_state.wait_neutral != 0U)
    {
        chassis_stop();
        if ((vx == 0.0f) && (vy == 0.0f) && (wz == 0.0f))
        {
            chassis_state.wait_neutral = 0U;
        }
        return;
    }

    vx *= (float)cfg->fwd_cur;
    vy *= (float)cfg->strafe_cur;
    wz *= (float)cfg->spin_cur;
    chassis_state.dbg_vx_input = vx; chassis_state.dbg_vy_input = vy;

    chassis_state.dbg_vx = vx; chassis_state.dbg_vy = vy; chassis_state.dbg_wz = wz;

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
        chassis_state.dbg_cur[i] = cur[i];
    }

    /* TIM6 已给 1kHz 节拍，这里不再限速 */
    motor_chassis_send_current(cur);
}

/* PID驾驶当拍数据；不改变输入、计时或反馈的读取位置。 */
typedef struct
{
    float x_norm, y_norm;
    float vx_input, vy_input, vx, vy, wz;
    uint8_t sw2, spin_on, follow_on;
} chassis_drive_t;

static void chassis_pid_sync(const chassis_cfg_t *cfg)
{
    uint8_t i;
    if (chassis_state.pid_ok == 0U)
    {
        for (i = 0U; i < 4U; i++)
        {
            pid_init(&chassis_pid[i], cfg->kp, cfg->ki, cfg->kd,
                     cfg->out_lim, cfg->int_lim);
        }
        chassis_state.pid_ok = 1U;
    }

    for (i = 0U; i < 4U; i++)
    {
        chassis_pid[i].kp             = cfg->kp;
        chassis_pid[i].ki             = cfg->ki;
        chassis_pid[i].kd             = cfg->kd;
        chassis_pid[i].output_limit   = cfg->out_lim;
        chassis_pid[i].integral_limit = cfg->int_lim;
    }

}

/* 含原有故障/归中/驻车处理；返回0时本拍已走相应停止路径。 */
static uint8_t chassis_prepare_drive(const chassis_cfg_t *cfg, chassis_drive_t *drive)
{
    /* 校验先于PID初始化、同步和目标计算。 */
    if (chassis_pid_config_valid(cfg) == 0U)
    {
        chassis_fail(CHASSIS_CONTROL_FAULT_CONFIG);
        return 0U;
    }

    chassis_pid_sync(cfg);

    chassis_yaw_update(cfg);
    if ((remote.online == 0U) ||
        ((drive->sw2 != INPUT_SW_UP) && (drive->sw2 != INPUT_SW_MID) && (drive->sw2 != INPUT_SW_DOWN)) ||
        ((cfg->move_frame != CHASSIS_FRAME_BODY) && (cfg->move_frame != CHASSIS_FRAME_GIMBAL)) ||
        ((cfg->move_frame == CHASSIS_FRAME_GIMBAL) && (chassis_state.yaw_valid == 0U)))
    {
        chassis_stop();
        return 0U;
    }

    drive->x_norm = input_axis(INPUT_AXIS_CHASSIS_X);
    drive->y_norm = input_axis(INPUT_AXIS_CHASSIS_Y);
    if ((drive->follow_on != 0U) &&
        (!chassis_range_valid(cfg->follow_kp, 0.0f, CHASSIS_GAIN_MAX) || (cfg->follow_kp <= 0.0f) ||
         !chassis_range_valid(cfg->follow_max_rpm, 0.0f, CHASSIS_COMMAND_MAX) || (cfg->follow_max_rpm <= 0.0f) ||
         !isfinite(cfg->follow_deadband_rad) || (cfg->follow_deadband_rad < 0.0f) ||
         (cfg->follow_deadband_rad >= (CHASSIS_TWO_PI * 0.5f))))
    {
        chassis_fail(CHASSIS_CONTROL_FAULT_CONFIG);
        return 0U;
    }
    if (chassis_state.park_state == CHASSIS_PARK_FAULT)
    {
        if ((cfg->park.enable == 0U) || (chassis_park_input_active() != 0U)) { chassis_park_reset(); }
        else { chassis_park_fault_stop(chassis_state.park_fault, chassis_state.park_brake_ms); return 0U; }
    }
    if (chassis_state.wait_neutral != 0U)
    {
        chassis_stop();
        if ((drive->x_norm == 0.0f) && (drive->y_norm == 0.0f) && (drive->spin_on == 0U))
        {
            chassis_state.wait_neutral = 0U;
        }
        return 0U;
    }

    chassis_park_update(cfg, HAL_GetTick());
    if (chassis_state.park_state == CHASSIS_PARK_FAULT)
    {
        chassis_park_fault_stop(chassis_state.park_fault, chassis_state.park_brake_ms);
        return 0U;
    }
    if (chassis_state.park_state != CHASSIS_PARK_DRIVE)
    {
        drive->x_norm = 0.0f;
        drive->y_norm = 0.0f;
        drive->follow_on = 0U;
    }

    return 1U;
}

static void chassis_transform_move(const chassis_cfg_t *cfg, chassis_drive_t *drive)
{
    /* 输入参考系中的平移指令，沿用现有转子 rpm 尺度。 */
    drive->vx_input = drive->x_norm * cfg->max_rpm;
    drive->vy_input = drive->y_norm * cfg->max_rpm;
    chassis_state.dbg_vx_input = drive->vx_input; chassis_state.dbg_vy_input = drive->vy_input;
    drive->vx = drive->vx_input;
    drive->vy = drive->vy_input;
    if (cfg->move_frame == CHASSIS_FRAME_GIMBAL)
    {
        float yaw_rel_rad = chassis_state.yaw_rel_rad;
        float sin_yaw = sinf(yaw_rel_rad);
        float cos_yaw = cosf(yaw_rel_rad);

        /* 相对角左转为正；按现有底盘侧向约定用负角变换，避免左右镜像。 */
        drive->vx =  cos_yaw * drive->vx_input + sin_yaw * drive->vy_input;
        drive->vy = -sin_yaw * drive->vx_input + cos_yaw * drive->vy_input;
    }
}

static void chassis_select_rotation(const chassis_cfg_t *cfg, chassis_drive_t *drive)
{
    /* 小陀螺优先；其余时候按实际角差跟随。
       本车实测正 drive->wz 使左正的相对角增大；跟随取负号，才会向零角差收敛。 */
    drive->wz = 0.0f;
    if (drive->spin_on != 0U)
    {
        drive->wz = cfg->spin_fixed_rpm;
    }
    else if ((drive->follow_on != 0U) &&
             (fabsf(chassis_state.yaw_rel_rad) > cfg->follow_deadband_rad))
    {
        drive->wz = -cfg->follow_kp * chassis_state.yaw_rel_rad;
        if (drive->wz >  cfg->follow_max_rpm) { drive->wz =  cfg->follow_max_rpm; }
        if (drive->wz < -cfg->follow_max_rpm) { drive->wz = -cfg->follow_max_rpm; }
        chassis_state.follow_active = 1U;
    }
    chassis_state.dbg_vx = drive->vx; chassis_state.dbg_vy = drive->vy; chassis_state.dbg_wz = drive->wz;

}

static uint8_t chassis_allocate_wheels(const chassis_cfg_t *cfg, chassis_drive_t *drive, float tgt[4])
{
    float maxabs = 0.0f;
    float scale = 1.0f;
    float move_scale = 1.0f;
    uint8_t i;
    if (drive->spin_on != 0U)
    {
        float move_rpm = fabsf(drive->vx) + fabsf(drive->vy);
        float available_rpm = cfg->max_rpm - fabsf(drive->wz);

        /* 自转本身已超轮速能力，不能假装仍能保持该目标。 */
        if (available_rpm < 0.0f) { chassis_fail(CHASSIS_CONTROL_FAULT_CONFIG); return 0U; }
        if (!isfinite(move_rpm)) { chassis_fail(CHASSIS_CONTROL_FAULT_NUMERIC); return 0U; }
        /* 先给自转预留轮速，余量只缩平移，保持驾驶方向。 */
        if (move_rpm > available_rpm)
        {
            move_scale = available_rpm / move_rpm;
            drive->vx *= move_scale;
            drive->vy *= move_scale;
        }
    }

    /* 逆运动学 -> 四轮目标（0~3 = FR/FL/RL/RR） */
    tgt[0] = drive->vx + drive->vy + drive->wz;    /* FR */
    tgt[1] = drive->vx - drive->vy - drive->wz;    /* FL */
    tgt[2] = drive->vx + drive->vy - drive->wz;    /* RL */
    tgt[3] = drive->vx - drive->vy + drive->wz;    /* RR */

    /* 普通驾驶仍等比缩放；小陀螺已预留旋转量，不能再一起缩小。 */
    for (i = 0U; i < 4U; i++)
    {
        if (!isfinite(tgt[i])) { chassis_fail(CHASSIS_CONTROL_FAULT_NUMERIC); return 0U; }
        float a = (tgt[i] < 0.0f) ? -tgt[i] : tgt[i];
        if (a > maxabs) { maxabs = a; }
    }
    if (maxabs > cfg->max_rpm)
    {
        if (drive->spin_on == 0U)
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
    chassis_state.pid_scale = scale;
    chassis_state.translation_scale = move_scale * scale;
    chassis_state.wz_applied = (tgt[0] - tgt[1] - tgt[2] + tgt[3]) * 0.25f;

    return 1U;
}

static uint8_t chassis_wheel_pid(const float tgt[4], int16_t cur[4])
{
    uint8_t i;
    for (i = 0U; i < 4U; i++)
    {
        float fb = (float)motor_chassis[i].fb.speed_rpm * (float)motor_dir[i];  /* 前进为正 */
        float command;

        chassis_state.pid_tar[i] = tgt[i];
        chassis_state.pid_fb [i] = fb;

        /* 电调掉线 -> 清 PID + 不给电流 */
        if ((motor_chassis[i].fb.last_ms == 0U) ||
            ((HAL_GetTick() - motor_chassis[i].fb.last_ms) > 100U))
        {
            pid_reset(&chassis_pid[i]);
            cur[i]       = 0;
            chassis_state.dbg_cur[i] = 0;
            continue;
        }

        if (!chassis_range_valid(chassis_pid[i].integral, -CHASSIS_INTEGRAL_MAX, CHASSIS_INTEGRAL_MAX) ||
            !chassis_range_valid(chassis_pid[i].last_error, -CHASSIS_PID_ERROR_MAX, CHASSIS_PID_ERROR_MAX))
        {
            chassis_fail(CHASSIS_CONTROL_FAULT_NUMERIC);
            return 0U;
        }
        command = pid_calc(&chassis_pid[i], tgt[i], fb) * (float)motor_dir[i];
        if (!isfinite(command)) { chassis_fail(CHASSIS_CONTROL_FAULT_NUMERIC); return 0U; }
        if (command >  CHASSIS_COMMAND_MAX) { command =  CHASSIS_COMMAND_MAX; }
        if (command < -CHASSIS_COMMAND_MAX) { command = -CHASSIS_COMMAND_MAX; }
        cur[i] = (int16_t)command;
        chassis_state.dbg_cur[i] = cur[i];
    }

    return 1U;
}

static void chassis_pid_run(const chassis_cfg_t *cfg)
{
    chassis_drive_t drive;
    float tgt[4];
    int16_t cur[4] = { 0, 0, 0, 0 };

    drive.sw2 = remote.sw2;
    drive.spin_on = (drive.sw2 == INPUT_SW_MID) ? 1U : 0U;
    drive.follow_on = (uint8_t)((drive.spin_on == 0U) && (cfg->follow_en != 0U) &&
                              (cfg->move_frame == CHASSIS_FRAME_GIMBAL));
    if (chassis_prepare_drive(cfg, &drive) == 0U) { return; }
    chassis_transform_move(cfg, &drive);
    chassis_select_rotation(cfg, &drive);
    if (chassis_allocate_wheels(cfg, &drive, tgt) == 0U) { return; }
    if (chassis_wheel_pid(tgt, cur) == 0U) { return; }
    motor_chassis_send_current(cur);
}

static void chassis_yaw_invalidate(void)
{
    chassis_state.yaw_valid = 0U;
    chassis_state.yaw_rel_rad = 0.0f;
    chassis_state.yaw_age_ms = UINT32_MAX;
}

static void chassis_rearm_on_change(chassis_mode_e mode, const chassis_cfg_t *cfg)
{
    static chassis_mode_e previous_mode = CHASSIS_MODE_STOP;
    static chassis_frame_e previous_frame = CHASSIS_FRAME_GIMBAL;
    static int16_t previous_zero = -1;
    static int8_t previous_dir = 0;
    static uint8_t previous_follow_en = 0U;
    /* 模式或几何配置变化后，先归中；PID 每拍使用这次复制的配置。 */
    if ((mode != previous_mode) ||
        ((mode == CHASSIS_MODE_PID_TEST) &&
         ((cfg->move_frame != previous_frame) ||
          (cfg->yaw_zero_ecd != previous_zero) || (cfg->yaw_ecd_dir != previous_dir) ||
          (cfg->follow_en != previous_follow_en))))
    {
        chassis_state.wait_neutral = 1U;
    }
    previous_mode = mode;
    previous_frame = cfg->move_frame;
    previous_zero = cfg->yaw_zero_ecd;
    previous_dir = cfg->yaw_ecd_dir;
    previous_follow_en = cfg->follow_en;

}

void chassis_run(void)
{
    chassis_mode_e mode = chassis_state.mode;
    const chassis_cfg_t cfg = chassis_cfg;
    chassis_state.follow_active = 0U;
    chassis_state.translation_scale = 0.0f;
    chassis_state.wz_applied = 0.0f;
    chassis_state.control_fault = CHASSIS_CONTROL_FAULT_NONE;

    if ((mode != CHASSIS_MODE_STOP) && (mode != CHASSIS_MODE_RC) && (mode != CHASSIS_MODE_PID_TEST))
    {
        chassis_fail(CHASSIS_CONTROL_FAULT_CONFIG);
        return;
    }

    /* WAIT/RUN 暂停底盘；超时已退出自动回正，可经下方有效性检查和归中后运行。
       保留超时告警，不伪装成回正成功；配置故障或未知状态仍停输出。 */
    if ((gimbal_state.home_state != GIMBAL_YAW_HOME_DONE) &&
        !((gimbal_state.home_state == GIMBAL_YAW_HOME_FAULT) &&
          (gimbal_state.home_fault == GIMBAL_YAW_HOME_FAULT_TIMEOUT)))
    {
        chassis_yaw_invalidate();
        chassis_stop();
        return;
    }

    chassis_rearm_on_change(mode, &cfg);

    switch (mode)
    {
    case CHASSIS_MODE_RC:
        chassis_park_reset();
        chassis_yaw_invalidate();
        chassis_rc_run(&cfg);
        break;

    case CHASSIS_MODE_PID_TEST:
        chassis_pid_run(&cfg);
        break;

    case CHASSIS_MODE_STOP:
    default:
        chassis_yaw_invalidate();
        chassis_stop();
        break;
    }
}
