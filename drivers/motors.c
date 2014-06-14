/*
 * motors.c
 *
 *  Created on: 31-08-2013
 *      Author: Korzo
 */

#include "motors.h"

#include <defs.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/pin_map.h>
#include <driverlib/gpio.h>
#include <driverlib/timer.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <stdbool.h>
#include <math.h>

//-----------------------------------------------------------------

#define THROTTLE_TO_PULSE(x)    (1000 + (uint16_t) min(max(0, (x)), THROTTLE_MAX))

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

static uint32_t cycle20ms;

static bool armed = false;

//-----------------------------------------------------------------

void motors_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    cycle20ms = SysCtlClockGet() / 50;

    uint32_t period = cycle20ms;
    uint8_t ext = period >> 16;
    period &= 0xFFFF;

    GPIOPinConfigure(MOTOR1_CFG);
    GPIOPinConfigure(MOTOR2_CFG);
    GPIOPinConfigure(MOTOR3_CFG);
    GPIOPinConfigure(MOTOR4_CFG);

    GPIOPinTypeTimer(MOTOR1_PORT, MOTOR1_PIN);
    GPIOPinTypeTimer(MOTOR2_PORT, MOTOR2_PIN);
    GPIOPinTypeTimer(MOTOR3_PORT, MOTOR3_PIN);
    GPIOPinTypeTimer(MOTOR4_PORT, MOTOR4_PIN);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
    TimerControlLevel(TIMER0_BASE, TIMER_BOTH, true);

    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
    TimerControlLevel(TIMER1_BASE, TIMER_BOTH, true);

    TimerPrescaleSet(TIMER0_BASE, TIMER_BOTH, ext);
    TimerLoadSet(TIMER0_BASE, TIMER_BOTH, period);

    TimerPrescaleSet(TIMER1_BASE, TIMER_BOTH, ext);
    TimerLoadSet(TIMER1_BASE, TIMER_BOTH, period);

    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    TimerEnable(TIMER1_BASE, TIMER_BOTH);

    motors_disarm();
}

//-----------------------------------------------------------------

bool motors_armed(void)
{
    return armed;
}

//-----------------------------------------------------------------

void motors_arm(void)
{
    armed = true;
}

//-----------------------------------------------------------------

void motors_disarm(void)
{
    armed = false;
    motors_set_throttle(0.0f, 0.0f, 0.0f, 0.0f);
}

//-----------------------------------------------------------------

void motors_set_throttle(float m1, float m2, float m3, float m4)
{
    motors_servo_pulse(MOTOR1_BASE, MOTOR1_TIMER, THROTTLE_TO_PULSE(m1));
    motors_servo_pulse(MOTOR2_BASE, MOTOR2_TIMER, THROTTLE_TO_PULSE(m2));
    motors_servo_pulse(MOTOR3_BASE, MOTOR3_TIMER, THROTTLE_TO_PULSE(m3));
    motors_servo_pulse(MOTOR4_BASE, MOTOR4_TIMER, THROTTLE_TO_PULSE(m4));
}

//-----------------------------------------------------------------

void motors_servo_pulse(uint32_t base, uint32_t timer, uint16_t pulse)
{
    uint32_t period = cycle20ms * pulse / 20000;
    uint8_t ext = period >> 16;
    period &= 0xFFFF;

    TimerMatchSet(base, timer, period);
    TimerPrescaleMatchSet(base, timer, ext);
}

