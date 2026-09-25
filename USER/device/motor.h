/* motor.h */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_send_current(const int16_t cur[4]);   /* cur[0]~cur[3] = 拨码 1~4 的电流 */

#endif