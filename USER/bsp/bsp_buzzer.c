/* bsp_buzzer.c */
#include "bsp_buzzer.h"
#include "tim.h"         /* CubeMX 的：htim12 */
#include "main.h"

#define BUZZER_DUTY_ON  185U    /* (369+1)/2 = 50% 占空比，最响 */

void bsp_buzzer_init(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0U);   /* 先静音（CCR=0 → 引脚常低） */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);           /* 启动 PWM 输出 */
}

void bsp_buzzer_beep(uint16_t ms)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, BUZZER_DUTY_ON);   /* 出声 */
    HAL_Delay(ms);                                                   /* 阻塞 ms 毫秒 */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0U);               /* 静音 */
}