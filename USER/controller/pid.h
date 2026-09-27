#ifndef __PID_H
#define __PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;

    float target;
    float feedback;

    float error;
    float last_error;
    float integral;         /* 积分累积 */
    float output;

    float integral_limit;   /* 积分限幅 */
    float output_limit;     /* 输出限幅 */
} pid_t;

void  pid_init(pid_t *pid, float kp, float ki, float kd,
               float out_lim, float int_lim);
void  pid_reset(pid_t *pid);
float pid_calc(pid_t *pid, float target, float feedback);

#endif