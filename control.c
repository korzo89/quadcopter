/*
 * control.c
 *
 *  Created on: 11-11-2013
 *      Author: Korzo
 */

#include "control.h"
#include "pid.h"
#include "motors.h"

//-----------------------------------------------------------------

QuadState state;
QuadState control;

PID_t pidPitch;
PID_t pidRoll;
PID_t pidYaw;

//-----------------------------------------------------------------

void controlInit(void)
{
    PIDReset(&pidPitch);
    PIDReset(&pidRoll);
}

//-----------------------------------------------------------------

void controlUpdate(float dt)
{
    float m1, m2, m3, m4;

    if (control.throttle > CONTROL_THROTTLE)
    {
        PIDUpdate(&pidPitch, state.pitch, dt);

        m1 = control.throttle - pidPitch.output + pidRoll.output + pidYaw.output;
        m2 = control.throttle + pidPitch.output + pidRoll.output - pidYaw.output;
        m3 = control.throttle + pidPitch.output - pidRoll.output + pidYaw.output;
        m4 = control.throttle - pidPitch.output - pidRoll.output - pidYaw.output;
    }
    else
    {
        m1 = control.throttle;
        m2 = control.throttle;
        m3 = control.throttle;
        m4 = control.throttle;
    }

    if (motorsArmed())
        motorsSetThrottle(m1, m2, m3, m4);
}

//-----------------------------------------------------------------

void controlThrottle(float throttle)
{
    if (throttle < 0.0)
        throttle = 0.0;
    else if (throttle > 1000.0)
        throttle = 1000.0;

    control.throttle = throttle;
}

//-----------------------------------------------------------------

void controlAttitude(float pitch, float roll, float yaw)
{
    control.pitch = pitch;
    control.roll = roll;
    control.yaw = yaw;
}

//-----------------------------------------------------------------

void controlSetPitchPID(float kp, float ki, float kd, float max, float min, float iMax)
{
    PIDConfig(&pidPitch, kp, ki, kd, max, min, iMax);
}

//-----------------------------------------------------------------

void controlSetRollPID(float kp, float ki, float kd, float max, float min, float iMax)
{
    PIDConfig(&pidRoll, kp, ki, kd, max, min, iMax);
}

//-----------------------------------------------------------------

void controlSetYawPID(float kp, float ki, float kd, float max, float min, float iMax)
{
    PIDConfig(&pidYaw, kp, ki, kd, max, min, iMax);
}
