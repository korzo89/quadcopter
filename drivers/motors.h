/*
 * motors.h
 *
 *  Created on: 31-08-2013
 *      Author: Korzo
 */

#ifndef MOTORS_H_
#define MOTORS_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------

#define THROTTLE_MAX    1000.0f

//-----------------------------------------------------------------

void motorsInit(void);

bool motorsArmed(void);
void motorsArm(void);
void motorsDisarm(void);

void motorsSetThrottle(float m1, float m2, float m3, float m4);

void motorsServoPulse(uint32_t base, uint32_t timer, uint16_t pulse);

//-----------------------------------------------------------------

#endif /* MOTORS_H_ */
