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
    PID_DERIV_ON_MEASUREMENT,
    PID_DERIV_ON_ERROR
};

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

typedef struct
{
    struct pid_params params;

    float setpoint;
    float output;
    float error;

    float prev_error;
    float prev_meas;
    float integral;
} pid_t;

//-----------------------------------------------------------------

result_t pid_init(pid_t *pid, const struct pid_params *params);
result_t pid_reset(pid_t *pid);

result_t pid_update(pid_t *pid, float meas, float dt, bool manual, float control);
result_t pid_update_auto(pid_t *pid, float meas, float dt);
result_t pid_update_manual(pid_t *pid, float meas, float dt, float control);

//-----------------------------------------------------------------

#endif /* PID_H_ */
