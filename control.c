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

bool enablePitch = false;
bool enableRoll = false;
bool enableYaw = false;

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
    float op, or, oy;

    if (control.throttle > CONTROL_THROTTLE)
    {
        PIDUpdate(&pidPitch, state.pitch, dt);

        op = enablePitch ? pidPitch.output : 0.0f;
        or = enableRoll ? pidRoll.output : 0.0f;
        oy = enableYaw ? pidYaw.output : 0.0f;

        m1 = control.throttle - op + or + oy;
        m2 = control.throttle + op + or - oy;
        m3 = control.throttle + op - or + oy;
        m4 = control.throttle - op - or - oy;
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
    else
        motorsSetThrottle(0.0f, 0.0f, 0.0f, 0.0f);
}

//-----------------------------------------------------------------

void controlThrottle(float throttle)
{
    if (throttle < 0.0f)
        throttle = 0.0f;
    else if (throttle > 1000.0f)
        throttle = 1000.0f;

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

void controlSetPitchPID(bool enable, float kp, float ki, float kd, float max, float min, float iMax)
{
    enablePitch = enable;
    PIDConfig(&pidPitch, kp, ki, kd, max, min, iMax);
}

//-----------------------------------------------------------------

void controlSetRollPID(bool enable, float kp, float ki, float kd, float max, float min, float iMax)
{
    enableRoll = enable;
    PIDConfig(&pidRoll, kp, ki, kd, max, min, iMax);
}

//-----------------------------------------------------------------

void controlSetYawPID(bool enable, float kp, float ki, float kd, float max, float min, float iMax)
{
    enableYaw = enable;
    PIDConfig(&pidYaw, kp, ki, kd, max, min, iMax);
}
