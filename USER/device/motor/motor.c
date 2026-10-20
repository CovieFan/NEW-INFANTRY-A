#include "motor.h"
#include "bsp_can.h"
#include "main.h"

#define MOTOR_CMD_ID  0x200U    
#define MOTOR_FB_BASE 0x201U       

volatile motor_t motor_std_fb[MOTOR_STD_NUM];
static void motor_gimbal_init(void);   /* 云台电机注册，定义在文件末尾 */

static void motor_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if ((data == 0) || (len < 8U))                            { return; }
    if ((id < MOTOR_FB_BASE) || (id > (MOTOR_FB_BASE + 3U)))  { return; }
    i = (uint16_t)(id - MOTOR_FB_BASE);      
    motor_std_fb[i].angle     = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    motor_std_fb[i].speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    motor_std_fb[i].current   = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
    motor_std_fb[i].temp      = data[6];
    motor_std_fb[i].online    = 1U;
    motor_std_fb[i].last_ms   = HAL_GetTick();
}

void motor_init(void)
{
    uint8_t i;
    for (i = 0U; i < MOTOR_STD_NUM; i++)
    {
        motor_std_fb[i].angle     = 0;
        motor_std_fb[i].speed_rpm = 0;
        motor_std_fb[i].current   = 0;
        motor_std_fb[i].temp      = 0U;
        motor_std_fb[i].online    = 0U;
        motor_std_fb[i].last_ms   = 0U;
    }
        bsp_can_register(BSP_CAN_1, 0x201U, 0x204U, motor_rx);   /* 底盘 4 个电调只在 CAN1 上 */
        motor_gimbal_init();                                      /* 云台 2 轴：0x205 / 0x206 */
        /*这一步是向CAN注册回调，这个注册回调的函数在bsp_can里面，因此在main函数里面进行初始化的时候需要先把bsp_can初始化，再把本文件初始化*/
}

void motor_send_current(const int16_t cur[MOTOR_STD_NUM])
{
    uint8_t data[8];
    uint8_t i;

    for (i = 0U; i < MOTOR_STD_NUM; i++)
    {
        int16_t c = cur[i];
        data[2U * i]      = (uint8_t)((uint16_t)c >> 8U);   /* 高字节在前 */
        data[2U * i + 1U] = (uint8_t)((uint16_t)c & 0xFFU);
    }
    (void)bsp_can_send_frame(BSP_CAN_1, MOTOR_CMD_ID, data, 8U);
}

volatile motor_t motor_gimbal_fb[GIMBAL_AXIS_COUNT];
volatile uint32_t       gimbal_rx_cnt[GIMBAL_AXIS_COUNT];
static   uint8_t        gimbal_raw[GIMBAL_AXIS_COUNT][8];   /* 最近一帧原始字节，供手工核对 */

static int16_t motor_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void motor_gimbal_store_raw(uint8_t idx, const uint8_t *data)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++) { gimbal_raw[idx][i] = data[i]; }
    motor_gimbal_fb[idx].online  = 1U;
    motor_gimbal_fb[idx].last_ms = HAL_GetTick();
    gimbal_rx_cnt[idx]++;
}

/* ---- YAW：GM6020，标准 RM 格式 ---- */
static void motor_gimbal_yaw_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len < 8U)) { return; }
    if (id != GIMBAL_YAW_FB_ID)    { return; }

    motor_gimbal_fb[GIMBAL_AXIS_YAW].angle        = motor_be16(&data[0]);
    motor_gimbal_fb[GIMBAL_AXIS_YAW].speed_rpm    = motor_be16(&data[2]);  /* 转速 */
    motor_gimbal_fb[GIMBAL_AXIS_YAW].current = motor_be16(&data[4]);  /* 实测电流 */
    motor_gimbal_fb[GIMBAL_AXIS_YAW].temp         = data[6];                /* 温度 */

    motor_gimbal_store_raw(GIMBAL_AXIS_YAW, data);
}

/* ---- PITCH：6623，专用格式（⚠️ 第 2-3 字节是电流，不是转速！）---- */
static void motor_gimbal_pitch_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len < 8U)) { return; }
    if (id != GIMBAL_PITCH_FB_ID)  { return; }

    motor_gimbal_fb[GIMBAL_AXIS_PITCH].angle        = motor_be16(&data[0]);  /* 角度也在偏移 0 */
    motor_gimbal_fb[GIMBAL_AXIS_PITCH].current = motor_be16(&data[2]);  /* 偏移2 = 实测电流 */
    /* ⚠️ 下面两行是"故意置 0"，因为这个型号的帧里根本没有这两个字段 */
    motor_gimbal_fb[GIMBAL_AXIS_PITCH].speed_rpm    = 0;
    motor_gimbal_fb[GIMBAL_AXIS_PITCH].temp         = 0U;

    motor_gimbal_store_raw(GIMBAL_AXIS_PITCH, data);
}

/* ---- PITCH：把命令值塞进 0x1FF 的槽位 1，发到 CAN2 ---- */
volatile int16_t  gimbal_tx_pitch      = 0;
volatile uint32_t gimbal_tx_pitch_ok   = 0U;
volatile uint32_t gimbal_tx_pitch_fail = 0U;

void motor_gimbal_send_pitch(int16_t cmd)
{
    uint8_t tx[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t off   = (uint8_t)(GIMBAL_PITCH_CMD_SLOT * 2U);   /* 槽位1 → 字节 2-3 */

    if (cmd >  (int16_t)GIMBAL_PITCH_CMD_MAX) { cmd =  (int16_t)GIMBAL_PITCH_CMD_MAX; }
    if (cmd < -(int16_t)GIMBAL_PITCH_CMD_MAX) { cmd = -(int16_t)GIMBAL_PITCH_CMD_MAX; }

    tx[off]      = (uint8_t)((uint16_t)cmd >> 8U);        /* 大端 */
    tx[off + 1U] = (uint8_t)((uint16_t)cmd & 0xFFU);

    if (bsp_can_send_frame(BSP_CAN_2, GIMBAL_PITCH_CMD_ID, tx, 8U) != 0U)  /* ★ CAN2 */
    {
        gimbal_tx_pitch_ok++;
    }
    else
    {
        gimbal_tx_pitch_fail++;
    }

    gimbal_tx_pitch = cmd;
}
static void motor_gimbal_init(void)
{
    uint8_t i;

    for (i = 0U; i < GIMBAL_AXIS_COUNT; i++)
    {
        motor_gimbal_fb[i].angle        = 0;
        motor_gimbal_fb[i].speed_rpm    = 0;
        motor_gimbal_fb[i].current = 0;
        motor_gimbal_fb[i].temp         = 0U;
        motor_gimbal_fb[i].online       = 0U;
        motor_gimbal_fb[i].last_ms      = 0U;
        gimbal_rx_cnt[i]        = 0U;
    }

    /* 型号差异：6020 有转速/温度；6623 两字段恒为 0（由各自的 rx 函数保证） */

    /* ★★ 只用"注册接收"，本步不发任何 CAN 指令 ★★
       一次注册同时验证三件事：① 双总线改造生效 ② 6020 的 0x205 没被 motor_rx 吞掉 ③ 6623 在 CAN2 上活着 */
    (void)bsp_can_register(BSP_CAN_1, GIMBAL_YAW_FB_ID,   GIMBAL_YAW_FB_ID,   motor_gimbal_yaw_rx);
    (void)bsp_can_register(BSP_CAN_2, GIMBAL_PITCH_FB_ID, GIMBAL_PITCH_FB_ID, motor_gimbal_pitch_rx);
}


volatile int16_t  gimbal_tx_yaw  = 0;
volatile uint32_t gimbal_tx_ok   = 0U;
volatile uint32_t gimbal_tx_fail = 0U;

/* ---- 把命令值塞进 0x1FF 的槽位 0，发到 CAN1 ----
   一帧 8 字节 = 4 个槽位（每个 int16 大端）。6020 只认自己那 2 个字节。 */
void motor_gimbal_send_yaw(int16_t cmd)
{
    uint8_t tx[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t off   = (uint8_t)(GIMBAL_YAW_CMD_SLOT * 2U);

    if (cmd >  (int16_t)GIMBAL_YAW_CMD_MAX) { cmd =  (int16_t)GIMBAL_YAW_CMD_MAX; }
    if (cmd < -(int16_t)GIMBAL_YAW_CMD_MAX) { cmd = -(int16_t)GIMBAL_YAW_CMD_MAX; }

    tx[off]      = (uint8_t)((uint16_t)cmd >> 8U);        /* 大端：高字节在前 */
    tx[off + 1U] = (uint8_t)((uint16_t)cmd & 0xFFU);

    if (bsp_can_send_frame(BSP_CAN_1, GIMBAL_YAW_CMD_ID, tx, 8U) != 0U)
    {
        gimbal_tx_ok++;
    }
    else
    {
        gimbal_tx_fail++;       /* ⭐ 以前这一路失败是"静默"的，现在看得见 */
    }
    gimbal_tx_yaw = cmd;
}
