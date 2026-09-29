/* motor.c */
#include "motor.h"
#include "bsp_can.h"
#include "main.h"

#define MOTOR_CMD_ID  0x200U    /* 0x200 帧 = 电机 1~4 的电流，一帧带 4 个 */
#define MOTOR_FB_BASE 0x201U        /* 0x201~0x204 = 电机 1~4 的反馈 */

volatile motor_t motor_fb[4];

/* ---- 收到反馈帧（在 CAN1 中断里被调用）：只解帧 + 存起来，别做别的 ---- */
static void motor_rx(uint32_t id, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((data == 0) || (len < 8U))                            { return; }
    if ((id < MOTOR_FB_BASE) || (id > (MOTOR_FB_BASE + 3U)))  { return; }

    i = (uint16_t)(id - MOTOR_FB_BASE);       /* 0~3 → 电机 1~4 */

    motor_fb[i].angle     = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    motor_fb[i].speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    motor_fb[i].current   = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
    motor_fb[i].temp      = data[6];
    motor_fb[i].online    = 1U;
    motor_fb[i].last_ms   = HAL_GetTick();
}



void motor_init(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        motor_fb[i].angle     = 0;
        motor_fb[i].speed_rpm = 0;
        motor_fb[i].current   = 0;
        motor_fb[i].temp      = 0U;
        motor_fb[i].online    = 0U;
        motor_fb[i].last_ms   = 0U;
    }
        bsp_can_register(BSP_CAN_1, 0x201U, 0x204U, motor_rx);   /* 底盘 4 个电调只在 CAN1 上 */
}

void motor_send_current(const int16_t cur[4])
{
    uint8_t data[8];
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        int16_t c = cur[i];
        data[2U * i]      = (uint8_t)((uint16_t)c >> 8U);   /* 高字节在前 */
        data[2U * i + 1U] = (uint8_t)((uint16_t)c & 0xFFU);
    }
    (void)bsp_can_send_frame(BSP_CAN_1, MOTOR_CMD_ID, data, 8U);
}