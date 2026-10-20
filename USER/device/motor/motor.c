#include "motor.h"
#include "bsp_can.h"
#include "main.h"

#define MOTOR_CMD_ID  0x200U    
#define MOTOR_FB_BASE 0x201U       

volatile motor_instance_t motor_chassis[MOTOR_CHASSIS_NUM];
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
    uint16_t i;
    if ((data == 0) || (len < 8U))                            { return; }
    if ((id < MOTOR_FB_BASE) || (id > (MOTOR_FB_BASE + 3U)))  { return; }
    i = (uint16_t)(id - MOTOR_FB_BASE);      
    motor_chassis[i].fb.angle     = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    motor_chassis[i].fb.speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    motor_chassis[i].fb.current   = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
    motor_chassis[i].fb.temp      = data[6];
    motor_chassis[i].fb.online    = 1U;
    motor_chassis[i].fb.last_ms   = HAL_GetTick();
}

void motor_chassis_init(void)
{
    motor_instances_reset(motor_chassis, MOTOR_CHASSIS_NUM);
    bsp_can_register(BSP_CAN_1, 0x201U, 0x204U, motor_chassis_rx);
}

void motor_chassis_send_current(const int16_t cur[MOTOR_CHASSIS_NUM])
{
    uint8_t data[8];
    uint8_t i;

    for (i = 0U; i < MOTOR_CHASSIS_NUM; i++)
    {
        int16_t c = cur[i];
        data[2U * i]      = (uint8_t)((uint16_t)c >> 8U);   /* 高字节在前 */
        data[2U * i + 1U] = (uint8_t)((uint16_t)c & 0xFFU);
        motor_chassis[i].last_cmd = c;
    }
    (void)bsp_can_send_frame(BSP_CAN_1, MOTOR_CMD_ID, data, 8U);
}

volatile motor_instance_t motor_gimbal[GIMBAL_AXIS_COUNT];
static uint8_t gimbal_raw[GIMBAL_AXIS_COUNT][8];

static const motor_cfg_t gimbal_cfg[GIMBAL_AXIS_COUNT] =
{
    { BSP_CAN_1, 0x205U, 0x1FFU, 0U, MOTOR_TYPE_RM_STD, 20000 },
    { BSP_CAN_2, 0x206U, 0x1FFU, 1U, MOTOR_TYPE_6623,   2000  },
};

static int16_t motor_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void motor_gimbal_store_raw(uint8_t idx, const uint8_t *data)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++) { gimbal_raw[idx][i] = data[i]; }
    motor_gimbal[idx].fb.online  = 1U;
    motor_gimbal[idx].fb.last_ms = HAL_GetTick();
    motor_gimbal[idx].rx_cnt++;
}

static void motor_gimbal_yaw_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len < 8U)) { return; }
    if (id != gimbal_cfg[GIMBAL_AXIS_YAW].fb_id) { return; }

    motor_gimbal[GIMBAL_AXIS_YAW].fb.angle     = motor_be16(&data[0]);
    motor_gimbal[GIMBAL_AXIS_YAW].fb.speed_rpm = motor_be16(&data[2]);
    motor_gimbal[GIMBAL_AXIS_YAW].fb.current   = motor_be16(&data[4]);
    motor_gimbal[GIMBAL_AXIS_YAW].fb.temp      = data[6];

    motor_gimbal_store_raw(GIMBAL_AXIS_YAW, data);
}

static void motor_gimbal_pitch_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len < 8U)) { return; }
    if (id != gimbal_cfg[GIMBAL_AXIS_PITCH].fb_id) { return; }

    motor_gimbal[GIMBAL_AXIS_PITCH].fb.angle     = motor_be16(&data[0]);
    motor_gimbal[GIMBAL_AXIS_PITCH].fb.current   = motor_be16(&data[2]);
    motor_gimbal[GIMBAL_AXIS_PITCH].fb.speed_rpm = 0;
    motor_gimbal[GIMBAL_AXIS_PITCH].fb.temp      = 0U;

    motor_gimbal_store_raw(GIMBAL_AXIS_PITCH, data);
}

static void motor_gimbal_send(uint8_t axis, int16_t cmd)
{
    const motor_cfg_t *cfg = &gimbal_cfg[axis];
    uint8_t tx[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t off   = (uint8_t)(cfg->slot * 2U);

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
        bsp_can_rx_t cb = (gimbal_cfg[i].type == MOTOR_TYPE_6623) ? motor_gimbal_pitch_rx
                                                                  : motor_gimbal_yaw_rx;
        (void)bsp_can_register(gimbal_cfg[i].bus, gimbal_cfg[i].fb_id, gimbal_cfg[i].fb_id, cb);
    }
}
