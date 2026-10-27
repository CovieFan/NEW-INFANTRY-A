#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>

/* 平移指令的参考方向；只用于 PID 驾驶模式，RC 电流测试始终使用车体系。 */
typedef enum
{
    CHASSIS_FRAME_BODY = 0,
    CHASSIS_FRAME_GIMBAL
} chassis_frame_e;

typedef enum
{
    CHASSIS_PARK_DRIVE = 0,
    CHASSIS_PARK_BRAKE,    /* 正常驻车：零轮速闭环，不是冻结电流或故障停机 */
    CHASSIS_PARK_HOLD,     /* 四轮停稳，允许YAW保持捕获的相对编码器角 */
    CHASSIS_PARK_FAULT     /* 制动超时/驻车轮反馈失效，停输出；恢复仍须归中 */
} chassis_park_state_e;

typedef enum
{
    CHASSIS_PARK_FAULT_NONE = 0,
    CHASSIS_PARK_FAULT_TIMEOUT,
    CHASSIS_PARK_FAULT_WHEEL
} chassis_park_fault_e;

typedef struct
{
    uint8_t  enable;
    uint32_t idle_ms;      /* 四轴连续回中且小陀螺关闭的等待时间 */
    float    stop_rpm;     /* 四轮停止判据，电机转子rpm */
    uint32_t settle_ms;    /* 四轮持续低于stop_rpm后才锁存YAW角度 */
    uint32_t brake_timeout_ms;
} chassis_park_cfg_t;

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
    chassis_park_cfg_t park;
} chassis_cfg_t;

typedef enum
{
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_RC,
    CHASSIS_MODE_PID_TEST
} chassis_mode_e;

/* 参数/计算异常诊断；其余许可仍看遥控、回正状态、反馈和wait_neutral。 */
typedef enum
{
    CHASSIS_CONTROL_FAULT_NONE = 0,
    CHASSIS_CONTROL_FAULT_CONFIG,
    CHASSIS_CONTROL_FAULT_NUMERIC,
    CHASSIS_CONTROL_FAULT_PARK
} chassis_control_fault_e;

extern volatile chassis_cfg_t  chassis_cfg;

/* 运行时可观测状态：逐成员访问，禁止整体赋值 */
typedef struct
{
    /* ---- 模式与故障 ---- */
    chassis_mode_e          mode;
    chassis_control_fault_e control_fault;
    uint8_t  wait_neutral;      /* 1 = 等待平移归中且关闭自转 */
    uint8_t  follow_active;     /* 1 = 自动跟随正在给出非零转向目标 */
    /* ---- 云台相对角（valid=0 时不得用于坐标转换）---- */
    float    yaw_rel_rad;       /* 左转为正，rad */
    uint32_t yaw_age_ms;
    uint8_t  yaw_valid;
    /* ---- 驻车 ---- */
    chassis_park_state_e park_state;
    chassis_park_fault_e park_fault;
    uint32_t park_idle_ms;
    uint32_t park_brake_ms;
    uint32_t park_stable_ms;
    int16_t  park_yaw_ecd;      /* HOLD 时有效，退出后为 -1 */
    int8_t   park_yaw_dir;
    /* ---- 输出诊断（每帧重算）---- */
    float    translation_scale; /* PID 平移实际缩放比例；停止/RC 模式为 0 */
    float    wz_applied;        /* 四轮目标反算的旋转分量，转子 rpm 尺度 */
    float    pid_tar[4];        /* 四轮目标（前进为正） */
    float    pid_fb[4];         /* 四轮反馈（前进为正） */
    float    pid_scale;
    uint8_t  pid_ok;
    /* ---- 调试 ---- */
    int16_t  dbg_cur[4];
    float    dbg_vx;
    float    dbg_vy;
    float    dbg_wz;
    float    dbg_vx_input;      /* 转换前指令；PID 模式单位为转子 rpm */
    float    dbg_vy_input;
} chassis_state_t;

extern volatile chassis_state_t chassis_state;

void chassis_run(void);

#endif /* CHASSIS_H */
