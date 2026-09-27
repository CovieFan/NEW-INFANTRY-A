#include "pid.h"

void pid_init(pid_t *p, float kp, float ki, float kd,
              float out_lim, float int_lim)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->output_limit = out_lim;
    p->integral_limit = int_lim;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->target   = 0.0f;
    p->feedback = 0.0f;
    p->error      = 0.0f;
    p->last_error = 0.0f;
    p->integral = 0.0f;
    p->output   = 0.0f;
}

float pid_calc(pid_t *p, float target, float feedback)
{
    p->target   = target;
    p->feedback = feedback;
    p->error    = target - feedback;

    /* 积分累积 + 限幅（防积分饱和：误差一直存在时积分不会无限涨） */
    p->integral += p->error;
    if (p->integral > p->integral_limit)
    {
        p->integral = p->integral_limit;
    }
    else if (p->integral < -p->integral_limit)
    {
        p->integral = -p->integral_limit;
    }

    p->output = p->kp * p->error
              + p->ki * p->integral
              + p->kd * (p->error - p->last_error);

    /* 输出限幅（本工程 = 限制电流上限，安全关键） */
    if (p->output > p->output_limit)
    {
        p->output = p->output_limit;
    }
    else if (p->output < -p->output_limit)
    {
        p->output = -p->output_limit;
    }

    p->last_error = p->error;
    return p->output;
}