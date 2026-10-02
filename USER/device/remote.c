#include "remote.h"
#include "usart.h"          /* CubeMX 的：huart1 */
#include "main.h"

#define RC_BUF_SIZE  64U    /* DBUS 一帧 18 字节，缓冲区开大点防溢出 */

static uint8_t s_rx_buf[RC_BUF_SIZE];

volatile uint16_t g_rc_bytes    = 0U;
volatile uint32_t g_rc_frames   = 0U;
volatile uint32_t g_rc_err_cnt  = 0U;
volatile uint32_t g_rc_err_code = 0U;

static void remote_unpack(const uint8_t *buff);   /* 前置声明：让上面的回调先认识它 */


void remote_init(void)
{
    /* 开启"空闲中断 + 中断接收"：总线一空闲就认为一帧收完，回调我们 */
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart1, s_rx_buf, sizeof(s_rx_buf));
    //让 huart1 开始收数据，收到一帧后自动喊回调函数来处理。
}

/* HAL 在 USART1 中断里回调：一帧收完（或总线空闲）时进这里 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart1) { return; }

    g_rc_bytes = Size;          /* ← 现象：这个数应该是 18 */
        if (Size == 18U)
    {
        remote_unpack(s_rx_buf);
    }
    g_rc_frames++;

    /* 必须立刻重新挂上接收，否则只收一帧就永远停了 */
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart1, s_rx_buf, sizeof(s_rx_buf));
}

/* 串口出错回调（校验错 PE / 帧错 FE / 溢出 ORE）—— 帧格式对不对，一眼看出 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) { return; }

    g_rc_err_cnt++;
    g_rc_err_code = huart->ErrorCode;

    /* 出错后也必须重挂：溢出会让接收永久死掉 */
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart1, s_rx_buf, sizeof(s_rx_buf));
}

void remote_update(void)
{
       uint32_t now = HAL_GetTick();

    /* 必须判 last_ms != 0：刚上电还没收到过任何帧时，不能算"在线" */
    /* 用减法比大小，而不是 now > last_ms + 150 —— 32 位 tick 约 49 天回绕，减法天然不怕 */
    if ((remote.last_ms != 0U) && ((now - remote.last_ms) <= 150U))
    {
        remote.online = 1U;
    }
    else
    {
        remote.online = 0U;
    }
}

volatile remote_t remote = {0};

/* DBUS 18 字节  */
static void remote_unpack(const uint8_t *buff)
{
    remote.ch0 = (int16_t)(((buff[0]       | (buff[1] << 8))                    & 0x07FFU) - 1024);
    remote.ch1 = (int16_t)((((buff[1] >> 3) | (buff[2] << 5))                   & 0x07FFU) - 1024);
    remote.ch2 = (int16_t)((((buff[2] >> 6) | (buff[3] << 2) | (buff[4] << 10)) & 0x07FFU) - 1024);
    remote.ch3 = (int16_t)((((buff[4] >> 1) | (buff[5] << 7))                   & 0x07FFU) - 1024);

    remote.sw1 = (uint8_t)(((buff[5] >> 4) & 0x03U));
    remote.sw2 = (uint8_t)((buff[5] >> 6) & 0x03U);

    remote.last_ms = HAL_GetTick();
}

