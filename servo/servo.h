/*
 * servo.h
 *
 *  Created on: 31-08-2013
 *      Author: Korzo
 */

#ifndef SERVO_H_
#define SERVO_H_

//-----------------------------------------------------------------

#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

//-----------------------------------------------------------------

#define SERVO0_BASE     TIMER0_BASE
#define SERVO0_TIMER    TIMER_A
#define SERVO0_PORT     GPIO_PORTB_BASE
#define SERVO0_PIN      GPIO_PIN_6
#define SERVO0_CFG      GPIO_PB6_T0CCP0

#define SERVO1_BASE     TIMER0_BASE
#define SERVO1_TIMER    TIMER_B
#define SERVO1_PORT     GPIO_PORTB_BASE
#define SERVO1_PIN      GPIO_PIN_7
#define SERVO1_CFG      GPIO_PB7_T0CCP1

#define SERVO2_BASE     TIMER1_BASE
#define SERVO2_TIMER    TIMER_A
#define SERVO2_PORT     GPIO_PORTB_BASE
#define SERVO2_PIN      GPIO_PIN_4
#define SERVO2_CFG      GPIO_PB4_T1CCP0

#define SERVO3_BASE     TIMER1_BASE
#define SERVO3_TIMER    TIMER_B
#define SERVO3_PORT     GPIO_PORTB_BASE
#define SERVO3_PIN      GPIO_PIN_5
#define SERVO3_CFG      GPIO_PB5_T1CCP1

//-----------------------------------------------------------------

void servoConfig();
void servoInit();

void servoSetPulse(uint32_t base, uint32_t timer, uint16_t pulse);

//-----------------------------------------------------------------

#endif /* SERVO_H_ */
