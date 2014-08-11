/*
 * pid.c
 *
 *  Created on: 11 cze 2014
 *      Author: Korzo
 */

#include "pid.h"
#include <string.h>

//-----------------------------------------------------------------

result_t pid_init(pid_t *pid, const struct pid_params *params)
{
    if (!pid || !params)
        return RES_ERR_BAD_PARAM;

    memcpy(&pid->params, params, sizeof(pid->params));
    return RES_OK;
}

//-----------------------------------------------------------------

result_t pid_reset(pid_t *pid)
{
    if (!pid)
        return RES_ERR_BAD_PARAM;

    pid->integral = 0;
    pid->error = 0;
    pid->prev_error = 0;
    pid->prev_meas = 0;
    pid->output = 0;

    return RES_OK;
}

//-----------------------------------------------------------------

result_t pid_update(pid_t *pid, float meas, float dt, bool manual, float control)
{
    if (!pid)
        return RES_ERR_BAD_PARAM;

    pid->error = pid->setpoint - meas;

    float err_i = pid->params.ki * pid->error * dt;
    pid->integral += err_i;

    float err_d;
    if (pid->params.deriv == PID_DERIV_ON_ERROR)
        err_d = pid->error - pid->prev_error;
    else
        err_d = -(meas - pid->prev_meas);
    err_d /= dt;

    // calculate final PID output
    float output = pid->params.kp * pid->error + pid->integral + pid->params.kd * err_d;
    // set manual control if requested
    float real_out = manual ? control : output;
    // output saturation
    if (real_out > pid->params.out_max)
        real_out = pid->params.out_max;
    else if (real_out < pid->params.out_min)
        real_out = pid->params.out_min;

    pid->output = real_out;

    // back-calculation for anti-windup and manual control
    float back = pid->params.kt * (real_out - output);
    pid->integral += back * dt;

    pid->prev_error = pid->error;
    pid->prev_meas = meas;

    return RES_OK;
}

//-----------------------------------------------------------------

result_t pid_update_auto(pid_t *pid, float value, float dt)
{
    return pid_update(pid, value, dt, false, 0.0f);
}

//-----------------------------------------------------------------

result_t pid_update_manual(pid_t *pid, float value, float dt, float control)
{
    return pid_update(pid, value, dt, true, control);
}
