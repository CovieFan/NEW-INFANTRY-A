#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
typedef struct
{
    int16_t  angle;         
    int16_t  speed_rpm;     
    int16_t  current;      
    uint8_t  temp;          
    uint8_t  online;       
    uint32_t last_ms;       
} motor_t;

#define MOTOR_STD_NUM   4U  /* 标准帧电机个数：当前 = 底盘 4 轮 */

extern volatile motor_t motor_std_fb[MOTOR_STD_NUM];

void motor_init(void);
void motor_send_current(const int16_t cur[MOTOR_STD_NUM]);


/* ============ 云台电机（与底盘共用 motor_t）============
   YAW   = GM6020，CAN1，反馈 0x205，命令 0x1FF 槽位 0
   PITCH = 6623，  CAN2，反馈 0x206，命令 0x1FF 槽位 1
   6020 = RM 标准帧；6623 = 专用帧（偏移 2 是电流） */

#define GIMBAL_AXIS_YAW       0U
#define GIMBAL_AXIS_PITCH     1U
#define GIMBAL_AXIS_COUNT     2U

#define GIMBAL_YAW_FB_ID      0x205U
#define GIMBAL_PITCH_FB_ID    0x206U

#define GIMBAL_YAW_CMD_ID     0x1FFU
#define GIMBAL_PITCH_CMD_ID   0x1FFU
#define GIMBAL_YAW_CMD_SLOT   0U
#define GIMBAL_PITCH_CMD_SLOT 1U

#define GIMBAL_YAW_CMD_MAX    20000   /* 0x1FF 电压帧量程 ±25000 */
#define GIMBAL_PITCH_CMD_MAX  2000    /* 6623 测试期量程 */

extern volatile motor_t  motor_gimbal_fb[GIMBAL_AXIS_COUNT];
extern volatile uint32_t gimbal_rx_cnt[GIMBAL_AXIS_COUNT];

extern volatile int16_t  gimbal_tx_yaw;
extern volatile uint32_t gimbal_tx_ok;
extern volatile uint32_t gimbal_tx_fail;
extern volatile int16_t  gimbal_tx_pitch;
extern volatile uint32_t gimbal_tx_pitch_ok;
extern volatile uint32_t gimbal_tx_pitch_fail;

void motor_gimbal_send_yaw(int16_t cmd);
void motor_gimbal_send_pitch(int16_t cmd);
#endif
