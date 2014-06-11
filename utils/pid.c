/*
 * pid.c
 *
 *  Created on: 11 cze 2014
 *      Author: Korzo
 */

#include "pid.h"
#include <string.h>

//-----------------------------------------------------------------

result_t pid_reset(pid_t *pid)
{
    if (!pid)
        return RES_ERR_BAD_PARAM;

    pid->integral = 0;
    pid->error = 0;
    pid->prev_error = 0;
    pid->output = 0;

    return RES_OK;
}

//-----------------------------------------------------------------

result_t pid_update(pid_t *pid, float value, float dt, bool manual, float control)
{
    if (!pid)
        return RES_ERR_BAD_PARAM;

    pid->error = pid->set_point - value;

    float err_i = pid->error;
    float err_d = (pid->error - pid->prev_error) / dt;

    // tracking mode
    if (manual)
        err_i += pid->kt * (control - pid->output);

    err_i *= dt;

    pid->integral += err_i;

    if (pid->integral > pid->int_max)
        pid->integral = pid->int_max;
    else if (pid->integral < pid->int_min)
        pid->integral = pid->int_min;

    float out_p = pid->kp * pid->error;
    float out_i = pid->ki * pid->integral;
    float out_d = pid->kd * err_d;

    pid->output = out_p + out_i + out_d;
    pid->prev_error = pid->error;

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
