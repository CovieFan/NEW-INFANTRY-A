#include "motor.h"
#include "bsp_can.h"
#include "main.h"

volatile motor_instance_t motor_chassis[MOTOR_CHASSIS_NUM];

static const motor_cfg_t motor_chassis_cfg[MOTOR_CHASSIS_NUM] =
{
    { BSP_CAN_1, 0x201U, 0x200U, 0U, MOTOR_TYPE_RM_STD, 16384 },  /* FR */
    { BSP_CAN_1, 0x202U, 0x200U, 1U, MOTOR_TYPE_RM_STD, 16384 },  /* FL */
    { BSP_CAN_1, 0x203U, 0x200U, 2U, MOTOR_TYPE_RM_STD, 16384 },  /* RL */
    { BSP_CAN_1, 0x204U, 0x200U, 3U, MOTOR_TYPE_RM_STD, 16384 },  /* RR */
};

static void motor_instances_reset(volatile motor_instance_t *inst, uint8_t n)
{
    uint8_t i;
    for (i = 0U; i < n; i++)
    {
        inst[i].fb.angle     = 0;
        inst[i].fb.speed_rpm = 0;
        inst[i].fb.current   = 0;
        inst[i].fb.temp      = 0U;
        inst[i].fb.online    = 0U;
        inst[i].fb.last_ms   = 0U;
        inst[i].last_cmd     = 0;
        inst[i].tx_ok        = 0U;
        inst[i].tx_err       = 0U;
        inst[i].rx_cnt       = 0U;
    }
}

static void motor_chassis_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    uint8_t i;

    if ((data == 0) || (len < 8U)) { return; }

    for (i = 0U; i < MOTOR_CHASSIS_NUM; i++)
    {
        if (motor_chassis_cfg[i].fb_id != id) { continue; }

        motor_chassis[i].fb.angle     = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
        motor_chassis[i].fb.speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
        motor_chassis[i].fb.current   = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
        motor_chassis[i].fb.temp      = data[6];
        motor_chassis[i].fb.online    = 1U;
        motor_chassis[i].fb.last_ms   = HAL_GetTick();
        motor_chassis[i].rx_cnt++;
        return;
    }
}

void motor_chassis_init(void)
{
    motor_instances_reset(motor_chassis, MOTOR_CHASSIS_NUM);
    bsp_can_register(motor_chassis_cfg[0].bus,
                     motor_chassis_cfg[0].fb_id,
                     motor_chassis_cfg[MOTOR_CHASSIS_NUM - 1U].fb_id,
                     motor_chassis_rx);
}

void motor_chassis_send_current(const int16_t cur[MOTOR_CHASSIS_NUM])
{
    uint8_t data[8] = { 0 };
    uint8_t i;

    for (i = 0U; i < MOTOR_CHASSIS_NUM; i++)
    {
        int16_t c   = cur[i];
        uint8_t off = (uint8_t)(motor_chassis_cfg[i].slot * 2U);

        if (c >  motor_chassis_cfg[i].out_max) { c =  motor_chassis_cfg[i].out_max; }
        if (c < -motor_chassis_cfg[i].out_max) { c = -motor_chassis_cfg[i].out_max; }

        data[off]      = (uint8_t)((uint16_t)c >> 8U);
        data[off + 1U] = (uint8_t)((uint16_t)c & 0xFFU);
        motor_chassis[i].last_cmd = c;
    }
    (void)bsp_can_send_frame(motor_chassis_cfg[0].bus, motor_chassis_cfg[0].cmd_id, data, 8U);
}

volatile motor_instance_t motor_gimbal[GIMBAL_AXIS_COUNT];

static const motor_cfg_t motor_gimbal_cfg[GIMBAL_AXIS_COUNT] =
{
    { BSP_CAN_1, 0x205U, 0x1FFU, 0U, MOTOR_TYPE_RM_STD, 20000 },
    { BSP_CAN_2, 0x206U, 0x1FFU, 1U, MOTOR_TYPE_6623,   2000  },
};

static int16_t motor_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void motor_gimbal_mark_online(uint8_t idx)
{
    motor_gimbal[idx].fb.online  = 1U;
    motor_gimbal[idx].fb.last_ms = HAL_GetTick();
    motor_gimbal[idx].rx_cnt++;
}

static void motor_gimbal_yaw_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len < 8U)) { return; }
    if (id != motor_gimbal_cfg[GIMBAL_AXIS_YAW].fb_id) { return; }

    motor_gimbal[GIMBAL_AXIS_YAW].fb.angle     = motor_be16(&data[0]);
    motor_gimbal[GIMBAL_AXIS_YAW].fb.speed_rpm = motor_be16(&data[2]);
    motor_gimbal[GIMBAL_AXIS_YAW].fb.current   = motor_be16(&data[4]);
    motor_gimbal[GIMBAL_AXIS_YAW].fb.temp      = data[6];

    motor_gimbal_mark_online(GIMBAL_AXIS_YAW);
}

static void motor_gimbal_pitch_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len < 8U)) { return; }
    if (id != motor_gimbal_cfg[GIMBAL_AXIS_PITCH].fb_id) { return; }

    motor_gimbal[GIMBAL_AXIS_PITCH].fb.angle     = motor_be16(&data[0]);
    motor_gimbal[GIMBAL_AXIS_PITCH].fb.current   = motor_be16(&data[2]);
    motor_gimbal[GIMBAL_AXIS_PITCH].fb.speed_rpm = 0;
    motor_gimbal[GIMBAL_AXIS_PITCH].fb.temp      = 0U;

    motor_gimbal_mark_online(GIMBAL_AXIS_PITCH);
}

static void motor_gimbal_send(uint8_t axis, int16_t cmd)
{
    const motor_cfg_t *cfg = &motor_gimbal_cfg[axis];
    uint8_t tx[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t off   = (uint8_t)(cfg->slot * 2U);

    /* 反馈超时 -> 强制 0（与底盘同判据：从未上线 或 100 ms 未更新） */
    if ((motor_gimbal[axis].fb.last_ms == 0U) ||
        ((HAL_GetTick() - motor_gimbal[axis].fb.last_ms) > 100U))
    {
        cmd = 0;
    }

    if (cmd >  cfg->out_max) { cmd =  cfg->out_max; }
    if (cmd < -cfg->out_max) { cmd = -cfg->out_max; }

    tx[off]      = (uint8_t)((uint16_t)cmd >> 8U);
    tx[off + 1U] = (uint8_t)((uint16_t)cmd & 0xFFU);

    if (bsp_can_send_frame(cfg->bus, cfg->cmd_id, tx, 8U) != 0U)
    {
        motor_gimbal[axis].tx_ok++;
    }
    else
    {
        motor_gimbal[axis].tx_err++;
    }

    motor_gimbal[axis].last_cmd = cmd;
}

void motor_gimbal_send_yaw(int16_t cmd)   { motor_gimbal_send(GIMBAL_AXIS_YAW, cmd); }
void motor_gimbal_send_pitch(int16_t cmd) { motor_gimbal_send(GIMBAL_AXIS_PITCH, cmd); }

void motor_gimbal_init(void)
{
    uint8_t i;

    motor_instances_reset(motor_gimbal, GIMBAL_AXIS_COUNT);

    for (i = 0U; i < GIMBAL_AXIS_COUNT; i++)
    {
        bsp_can_rx_t cb = (motor_gimbal_cfg[i].type == MOTOR_TYPE_6623) ? motor_gimbal_pitch_rx
                                                                  : motor_gimbal_yaw_rx;
        (void)bsp_can_register(motor_gimbal_cfg[i].bus, motor_gimbal_cfg[i].fb_id, motor_gimbal_cfg[i].fb_id, cb);
    }
}
