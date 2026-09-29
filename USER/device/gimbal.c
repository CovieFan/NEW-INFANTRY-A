/* gimbal.c —— 云台设备层（步5a：只收不发）
   分工：bsp_can 只管搬字节；这里才知道"0x205 是 yaw 的 6020"。 */
#include "gimbal.h"
#include "bsp_can.h"
#include "main.h"          /* HAL_GetTick() */

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