/*
 * servo.h
 *
 *  Created on: 31-08-2013
 *      Author: Korzo
 */

#ifndef MOTORS_H_
#define MOTORS_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

//-----------------------------------------------------------------

#define THROTTLE_MAX    1000.0f

#define MOTOR1_BASE     TIMER0_BASE
#define MOTOR1_TIMER    TIMER_A
#define MOTOR1_PORT     GPIO_PORTB_BASE
#define MOTOR1_PIN      GPIO_PIN_6
#define MOTOR1_CFG      GPIO_PB6_T0CCP0

#define MOTOR2_BASE     TIMER0_BASE
#define MOTOR2_TIMER    TIMER_B
#define MOTOR2_PORT     GPIO_PORTB_BASE
#define MOTOR2_PIN      GPIO_PIN_7
#define MOTOR2_CFG      GPIO_PB7_T0CCP1

#define MOTOR3_BASE     TIMER1_BASE
#define MOTOR3_TIMER    TIMER_A
#define MOTOR3_PORT     GPIO_PORTB_BASE
#define MOTOR3_PIN      GPIO_PIN_4
#define MOTOR3_CFG      GPIO_PB4_T1CCP0

#define MOTOR4_BASE     TIMER1_BASE
#define MOTOR4_TIMER    TIMER_B
#define MOTOR4_PORT     GPIO_PORTB_BASE
#define MOTOR4_PIN      GPIO_PIN_5
#define MOTOR4_CFG      GPIO_PB5_T1CCP1

//-----------------------------------------------------------------

void motorsConfig(void);
void motorsInit(void);

bool motorsArmed(void);
void motorsArm(void);
void motorsDisarm(void);

void motorsSetThrottle(float m1, float m2, float m3, float m4);

void motorsServoPulse(uint32_t base, uint32_t timer, uint16_t pulse);

//-----------------------------------------------------------------

#endif /* MOTORS_H_ */
