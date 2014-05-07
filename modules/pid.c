/*
 * pid.c
 *
 *  Created on: 16-10-2013
 *      Author: Korzo
 */

#include "pid.h"

//-----------------------------------------------------------------

void PIDConfig(PID_t *pid, float kp, float ki, float kd, float max, float min, float iMax)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->maxOut = max;
    pid->minOut = min;
    pid->maxInt = iMax;
}

//-----------------------------------------------------------------

void PIDReset(PID_t *pid)
{
    pid->output = 0.0f;
    pid->prevErr = 0.0f;
    pid->integral = 0.0f;
}

//-----------------------------------------------------------------

void PIDUpdate(PID_t *pid, float val, float dt)
{
    float error = pid->setPoint - val;
    float ui = pid->integral + (error * dt);
    float ud = (error - pid->prevErr) / dt;
    float out = pid->kp * error + pid->ki * ui + pid->kd * ud;

    if (out > pid->maxOut || out < pid->minOut || ui > pid->maxInt)
    {
        ui = pid->integral;
        out = pid->kp * error + pid->ki * ui + pid->kd * ud;
    }

    if (out > pid->maxOut)
        pid->output = pid->maxOut;
    else if (out < pid->minOut)
        pid->output = pid->minOut;
    else
        pid->output = out;

    pid->prevErr = error;
}
