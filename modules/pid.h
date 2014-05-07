/*
 * pid.h
 *
 *  Created on: 16-10-2013
 *      Author: Korzo
 */

#ifndef PID_H_
#define PID_H_

//-----------------------------------------------------------------

typedef struct
{
    float kp;
    float ki;
    float kd;
    float maxOut;
    float minOut;
    float maxInt;
    float setPoint;
    float output;
    float integral;
    float prevErr;
} PID_t;

//-----------------------------------------------------------------

void PIDConfig(PID_t *pid, float kp, float ki, float kd, float max, float min, float iMax);
void PIDReset(PID_t *pid);

void PIDUpdate(PID_t *pid, float val, float dt);

//-----------------------------------------------------------------

#endif /* PID_H_ */
