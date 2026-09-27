
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* ================= 遥控开环控制 ================= */
#define CHASSIS_RC_ENABLE   1U      /* ★总开关：0 = 遥控控制关闭（保险丝） */

#define CHASSIS_FWD_CUR     2500    /* 摇杆推到底时的"前进"电流 */
#define CHASSIS_STRAFE_CUR  2500    /*                            "平移"电流 */
#define CHASSIS_SPIN_CUR    2500    /*                            "自转"电流 */
#define CHASSIS_CUR_MAX     4500    /* 单轮电流硬上限（±16384 ≈ ±20A） */

/* ★ 通道映射：下标 0=右水平 1=右竖直 2=左水平 3=左竖直 */
#define RC_CH_VX        3          /* 前进/后退：左摇杆纵向 ch3 */
#define RC_CH_VY        2          /* 左右平移：左摇杆横向 ch2（不想用就改 0xFF） */
#define RC_CH_WZ        0           /* 转向    ：右摇杆横向 ch0 */
                                    /* 右摇杆纵向 ch1 留空 → 以后给云台 */

/* ★ 方向符号：推杆方向反了就翻这里（不是 bug，是摇杆极性） */
#define RC_SIGN_VX     (+1)         /* 推"往上"却后退        → 改 (-1) */
#define RC_SIGN_VY     (+1)         /* 推"往左"却往右平移    → 改 (-1) */
#define RC_SIGN_WZ     (+1)         /* 推"往左"却右转        → 改 (-1) */

#define RC_DEADBAND     30          /* 摇杆死区，防中位漂移 */


void chassis_run(void);      /* 主循环每圈调一次 */

#endif /* CHASSIS_H */


