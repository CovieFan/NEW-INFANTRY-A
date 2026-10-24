#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>

/* 平移指令的参考方向；只用于 PID 驾驶模式，RC 电流测试始终使用车体系。 */
typedef enum
{
    CHASSIS_FRAME_BODY = 0,
    CHASSIS_FRAME_GIMBAL
} chassis_frame_e;

typedef struct
{
    int16_t fwd_cur;
    int16_t strafe_cur;
    int16_t spin_cur;
    int16_t cur_max;
    float   spin_fixed_rpm;

    uint8_t follow_en;           /* PID 云台坐标驾驶：非小陀螺时车头跟随枪口 */
    float   follow_kp;           /* 相对角 -> 四轮解算中的转向目标，rpm/rad */
    float   follow_max_rpm;      /* 自动跟随转向目标上限，沿用 spin_fixed_rpm 尺度 */
    float   follow_deadband_rad; /* 对齐死区，rad，避免小角差反复修正 */

    float   max_rpm;
    float   kp;
    float   ki;
    float   kd;
    float   out_lim;
    float   int_lim;

    chassis_frame_e move_frame;
    int16_t  yaw_zero_ecd;    /* 枪口与车头对齐时的编码器值；-1 = 未标定 */
    int8_t   yaw_ecd_dir;     /* 左转时编码器增加填 +1，减小填 -1；0 = 未标定 */
    uint32_t yaw_timeout_ms;  /* YAW 反馈最大允许年龄，必须大于 0 */
} chassis_cfg_t;

typedef enum
{
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_RC,
    CHASSIS_MODE_PID_TEST
} chassis_mode_e;

extern volatile chassis_cfg_t  chassis_cfg;
extern volatile chassis_mode_e chassis_mode;

/* Watch：相对角左转为正，单位 rad；valid=0 时不得用于坐标转换。 */
extern volatile float    chassis_yaw_rel_rad;
extern volatile uint32_t chassis_yaw_age_ms;
extern volatile uint8_t  chassis_yaw_valid;
extern volatile uint8_t  chassis_wait_neutral; /* 1 = 等待平移归中且关闭自转 */
extern volatile uint8_t  chassis_follow_active; /* 1 = 自动跟随正在给出非零转向目标 */
extern volatile float    dbg_vx_input;        /* 转换前指令；PID 模式单位为转子 rpm */
extern volatile float    dbg_vy_input;

void chassis_run(void);

#endif /* CHASSIS_H */
