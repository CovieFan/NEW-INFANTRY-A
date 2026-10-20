#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>

typedef struct
{
    int16_t fwd_cur;
    int16_t strafe_cur;
    int16_t spin_cur;
    int16_t cur_max;
    float   spin_fixed_rpm;

    float   max_rpm;
    float   kp;
    float   ki;
    float   kd;
    float   out_lim;
    float   int_lim;
} chassis_cfg_t;

typedef enum
{
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_RC,
    CHASSIS_MODE_PID_TEST
} chassis_mode_e;

extern volatile chassis_cfg_t  chassis_cfg;
extern volatile chassis_mode_e chassis_mode;

void chassis_run(void);

#endif /* CHASSIS_H */
