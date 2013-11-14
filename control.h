/*
 * control.h
 *
 *  Created on: 11-11-2013
 *      Author: Korzo
 */

#ifndef CONTROL_H_
#define CONTROL_H_

//-----------------------------------------------------------------

#define CONTROL_THROTTLE        100.0f

//-----------------------------------------------------------------

typedef struct
{
    float throttle;
    float pitch;
    float roll;
    float yaw;
} QuadState;

//-----------------------------------------------------------------

void controlInit(void);
void controlUpdate(float dt);

void controlThrottle(float throttle);
void controlAttitude(float pitch, float roll, float yaw);

void controlSetPitchPID(float kp, float ki, float kd, float max, float min, float iMax);
void controlSetRollPID(float kp, float ki, float kd, float max, float min, float iMax);
void controlSetYawPID(float kp, float ki, float kd, float max, float min, float iMax);

//-----------------------------------------------------------------

#endif /* CONTROL_H_ */
