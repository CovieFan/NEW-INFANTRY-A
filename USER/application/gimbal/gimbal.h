#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef enum
{
    GIMBAL_YAW_HOME_WAIT = 0, /* 等待有效反馈、IMU 标定和遥控在线 */
    GIMBAL_YAW_HOME_RUN,
    GIMBAL_YAW_HOME_DONE,     /* 本次上电已完成，不因普通掉线重新回正 */
    GIMBAL_YAW_HOME_FAULT     /* 自动回正退出；超时可归中后手动接管，底盘独立检查放行 */
} gimbal_yaw_home_state_e;

typedef enum
{
    GIMBAL_YAW_HOME_FAULT_NONE = 0,
    GIMBAL_YAW_HOME_FAULT_CONFIG,
    GIMBAL_YAW_HOME_FAULT_TIMEOUT
} gimbal_yaw_home_fault_e;

/* Watch 位图；PITCH 的 IMU 位表示IMU启动流程尚未结束。 */
typedef enum
{
    GIMBAL_INHIBIT_DISABLED  = 1U,
    GIMBAL_INHIBIT_REMOTE    = 2U,
    GIMBAL_INHIBIT_IMU       = 4U,
    GIMBAL_INHIBIT_FEEDBACK  = 8U,
    GIMBAL_INHIBIT_NONFINITE = 16U, /* 非有限值或超出宽松数值状态边界 */
    GIMBAL_INHIBIT_CONFIG   = 32U
} gimbal_inhibit_e;

typedef struct
{
    float    kp_ang;          /* 启动角差 -> 目标速度，独立于正常遥控外环 */
    float    speed_max;       /* 回正目标速度上限，rad/s */
    float    speed_min;       /* 窗口外的最小回正速度，rad/s，克服末端静摩擦 */
    float    out_lim;         /* 回正命令上限，另受原 YAW 输出限幅约束 */
    uint16_t tolerance_ecd;   /* 到位角差容差，编码器计数 */
    float    omega_tolerance; /* 到位速度容差，rad/s */
    uint32_t settle_ms;       /* 连续满足角差及速度容差的时间 */
    uint32_t timeout_ms;      /* 从首次开始回正算起，暂停不重新计时 */
} gimbal_yaw_home_cfg_t;

typedef struct
{
    uint8_t speed_mode;    /* 1 = 速控；0 = 位控 */
    uint8_t omega_ff_en;   /* 位控：速度前馈开关 */
    float   kp_w;          /* 速度环 P：命令值 / (rad/s) */
    float   kick;          /* 起步电流*/
    float   speed_max;     
    float   kp_ang;        
    float   ki_ang;        
    float   w_ref_lim;     
    float   int_lim;      
    float   tgt_rate;      /* 位控：摇杆积分速率 (rad/tick) */
    float   out_lim;       
    gimbal_yaw_home_cfg_t home;
} gimbal_yaw_cfg_t;

typedef struct
{
    int16_t level;         /* 云台水平时 motor_gimbal[1].fb.angle 的实测值 */
    float   kp;
    float   ki;
    float   kd;
    float   out_lim;       /* 必须 <= motor_gimbal_cfg[1].out_max */
    float   int_lim;
    float   dir;           /* 方向系数 +-1 */
} gimbal_pitch_cfg_t;

typedef struct
{
    uint8_t enable;      
    gimbal_yaw_cfg_t   yaw;
    gimbal_pitch_cfg_t pitch;
} gimbal_cfg_t;

extern volatile gimbal_cfg_t gimbal_cfg;
extern volatile gimbal_yaw_home_state_e yaw_home_state;
extern volatile gimbal_yaw_home_fault_e yaw_home_fault;
extern volatile float    yaw_home_err;        /*最短回正角差，rad，左转为正 */
extern volatile uint32_t yaw_home_elapsed_ms;
extern volatile uint32_t yaw_home_stable_ms;
extern volatile uint8_t  yaw_home_manual_ready; /* 超时后 YAW 杆归中，允许手动接管 */
extern volatile uint8_t  yaw_control_inhibit;
extern volatile uint8_t  pitch_control_inhibit;
extern volatile uint32_t yaw_feedback_age_ms;
extern volatile uint32_t pitch_feedback_age_ms;

void gimbal_run(void);

#endif
