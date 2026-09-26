
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* ===== TEMP: 4a 逐轮开环标定开关（测完改成 0U，并删掉本段）===== */
#define CHASSIS_OL_TEST   1U

#if (CHASSIS_OL_TEST != 0U)
extern volatile uint8_t g_chassis_ol_slot;      /* 当前格号；0xFF = 还没开始 */
extern volatile int16_t g_chassis_ol_cur[4];    /* 当前实际发出去的 4 个电流 */
#endif

void chassis_run(void);      /* 主循环每圈调一次 */

#endif /* CHASSIS_H */