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

typedef struct
{
    motor_t  fb;
    int16_t  last_cmd;
    uint32_t tx_ok;
    uint32_t tx_err;
    uint32_t rx_cnt;
} motor_instance_t;

typedef enum
{
    MOTOR_TYPE_RM_STD = 0,//RM标准反馈帧
    MOTOR_TYPE_6623   = 1//6623专用帧
} motor_type_e;

typedef struct
{
    uint8_t      bus;
    uint32_t     fb_id;
    uint32_t     cmd_id;
    uint8_t      slot;
    motor_type_e type;
    int16_t      out_max;
} motor_cfg_t;

#define MOTOR_CHASSIS_NUM   4U

extern volatile motor_instance_t motor_chassis[MOTOR_CHASSIS_NUM];
void motor_chassis_init(void);
void motor_chassis_send_current(const int16_t cur[MOTOR_CHASSIS_NUM]);

#define GIMBAL_AXIS_YAW       0U
#define GIMBAL_AXIS_PITCH     1U
#define GIMBAL_AXIS_COUNT     2U
extern volatile motor_instance_t motor_gimbal[GIMBAL_AXIS_COUNT];
void motor_gimbal_init(void);
void motor_gimbal_send_yaw(int16_t cmd);
void motor_gimbal_send_pitch(int16_t cmd);

#endif
