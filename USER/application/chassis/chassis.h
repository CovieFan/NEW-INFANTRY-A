#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>

#define CHASSIS_FWD_CUR     2500
#define CHASSIS_STRAFE_CUR  2500
#define CHASSIS_SPIN_CUR    2500
#define CHASSIS_CUR_MAX     4500

#define CHASSIS_PID_MAX_RPM  3000.0f
#define CHASSIS_PID_KP       15.0f
#define CHASSIS_PID_KI       0.0f
#define CHASSIS_PID_KD       0.0f
#define CHASSIS_PID_OUT_LIM  3000.0f
#define CHASSIS_PID_INT_LIM  3000.0f

#define CHASSIS_SPIN_FIXED_RPM  1000.0f

typedef enum
{
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_RC,
    CHASSIS_MODE_PID_TEST
} chassis_mode_e;

extern volatile chassis_mode_e chassis_mode;

void chassis_run(void);

#endif /* CHASSIS_H */
