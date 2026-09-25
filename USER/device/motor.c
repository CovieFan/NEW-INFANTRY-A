/* motor.c */
#include "motor.h"
#include "bsp_can.h"

#define MOTOR_CMD_ID  0x200U    /* 0x200 帧 = 电机 1~4 的电流，一帧带 4 个 */

void motor_init(void)
{
    /* 步2 还不需要清状态；步3 在这里注册 motor_rx */
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
    (void)bsp_can_send_frame(MOTOR_CMD_ID, data, 8U);
}