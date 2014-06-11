/*
 * pid.h
 *
 *  Created on: 11 cze 2014
 *      Author: Korzo
 */

#ifndef PID_H_
#define PID_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

typedef struct
{
    float set_point;
    float output;
    float error;
    float prev_error;
    float integral;
    float kp;
    float ki;
    float kd;
    float kt;
    float int_min;
    float int_max;
} pid_t;

//-----------------------------------------------------------------

result_t pid_reset(pid_t *pid);

result_t pid_update(pid_t *pid, float value, float dt, bool manual, float control);
result_t pid_update_auto(pid_t *pid, float value, float dt);
result_t pid_update_manual(pid_t *pid, float value, float dt, float control);

//-----------------------------------------------------------------

#endif /* PID_H_ */
