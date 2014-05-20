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

void motors_init(void);

bool motors_armed(void);
void motors_arm(void);
void motors_disarm(void);

void motors_set_throttle(float m1, float m2, float m3, float m4);

void motors_servo_pulse(uint32_t base, uint32_t timer, uint16_t pulse);

//-----------------------------------------------------------------

#endif /* MOTORS_H_ */
