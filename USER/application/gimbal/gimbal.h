#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/* 云台轴控制（application 层）
   电机的反馈解析、时间戳、命令组帧都在 device/motor；本文件只有控制。 */

/* ★总开关（保险丝）：0 = 云台恒发 0，不响应摇杆 */
#define GIMBAL_RAW_TEST_ENABLE  1U

void gimbal_run(void);   /* 1kHz 调用：YAW / PITCH 控制 + 发送 */

#endif
