
#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>

#define CHASSIS_RC_ENABLE   1U      
#define CHASSIS_FWD_CUR     2500    
#define CHASSIS_STRAFE_CUR  2500   
#define CHASSIS_SPIN_CUR    2500    
#define CHASSIS_CUR_MAX     4500    

#define CHASSIS_PID_TEST     1U          /* ★总开关：测完改回 0U */
#define CHASSIS_PID_MAX_RPM  3000.0f     /* 摇杆推到底的目标转速（转子 rpm）*/
#define CHASSIS_PID_KP       15.0f
#define CHASSIS_PID_KI       0.0f
#define CHASSIS_PID_KD       0.0f
#define CHASSIS_PID_OUT_LIM  3000.0f     
#define CHASSIS_PID_INT_LIM  3000.0f

#define CHASSIS_SPIN_FIXED_RPM  1000.0f


void chassis_run(void);      /* 主循环每圈调一次 */

#endif /* CHASSIS_H */


