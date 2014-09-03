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

enum pid_deriv_type
{
    PID_DERIV_ON_MEASUREMENT = 0,
    PID_DERIV_ON_ERROR
};

#define PID_DERIV_TYPE_META     "pid_deriv_type;PID_DERIV_;ON_MEASUREMENT;ON_ERROR"

struct pid_params
{
    float kp;
    float ki;
    float kd;
    float kt;
    float out_min;
    float out_max;
    enum pid_deriv_type deriv;
};

struct pid
{
    struct pid_params params;

    float setpoint;
    float output;
    float error;

    float prev_error;
    float prev_meas;
    float integral;
};

//-----------------------------------------------------------------

result_t pid_init(struct pid *pid, const struct pid_params *params);
result_t pid_reset(struct pid *pid);

result_t pid_update(struct pid *pid, float meas, float dt, bool manual, float control);
result_t pid_update_auto(struct pid *pid, float meas, float dt);
result_t pid_update_manual(struct pid *pid, float meas, float dt, float control);

//-----------------------------------------------------------------

#endif /* PID_H_ */
