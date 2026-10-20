#include "bsp_uart.h"
#include "usart.h"        /* CubeMX 的：huart8 */
#include <stdio.h>
#include <stdarg.h>

void BspPrintf(const char *fmt, ...)
{
    char buf[128];
    int  n;
    va_list ap;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > (int)(sizeof(buf) - 1)) { n = (int)(sizeof(buf) - 1); }   /* 截断了就只发实际有的 */
    if (n > 0)
    {
        HAL_UART_Transmit(&huart8, (uint8_t *)buf, (uint16_t)n, 20U);
    }
}