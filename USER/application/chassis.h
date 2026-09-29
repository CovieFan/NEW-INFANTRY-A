
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* ================= 遥控开环控制 ================= */
#define CHASSIS_RC_ENABLE   1U      /* ★总开关：0 = 遥控控制关闭（保险丝） */

#define CHASSIS_FWD_CUR     2500    /* 摇杆推到底时的"前进"电流 */
#define CHASSIS_STRAFE_CUR  2500    /*                            "平移"电流 */
#define CHASSIS_SPIN_CUR    2500    /*                            "自转"电流 */
#define CHASSIS_CUR_MAX     4500    /* 单轮电流硬上限（±16384 ≈ ±20A） */

/* 通道映射 / 方向符号 / 死区 已搬到 application/input.h（输入层） */

/* ================= 4c-2 四轮转速闭环（遥控驱动） ================= */
#define CHASSIS_PID_TEST     1U          /* ★总开关：测完改回 0U */
#define CHASSIS_PID_MAX_RPM  3000.0f     /* 摇杆推到底的目标转速（转子 rpm）*/

#define CHASSIS_PID_KP       15.0f
#define CHASSIS_PID_KI       0.0f
#define CHASSIS_PID_KD       0.0f
#define CHASSIS_PID_OUT_LIM  3000.0f     /* 单轮电流上限 */
#define CHASSIS_PID_INT_LIM  3000.0f


void chassis_run(void);      /* 主循环每圈调一次 */

#endif /* CHASSIS_H */


