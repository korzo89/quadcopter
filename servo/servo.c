/*
 * servo.c
 *
 *  Created on: 31-08-2013
 *      Author: Korzo
 */

#include "servo.h"

#include <stdbool.h>

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

//-----------------------------------------------------------------

static uint32_t cycle20ms;

//-----------------------------------------------------------------

void servoConfig()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    cycle20ms = SysCtlClockGet() / 50;

    uint32_t period = cycle20ms;
    uint8_t ext = period >> 16;
    period &= 0xFFFF;

    GPIOPinConfigure(SERVO0_CFG);
    GPIOPinConfigure(SERVO1_CFG);
    GPIOPinConfigure(SERVO2_CFG);
    GPIOPinConfigure(SERVO3_CFG);

    GPIOPinTypeTimer(SERVO0_PORT, SERVO0_PIN);
    GPIOPinTypeTimer(SERVO1_PORT, SERVO1_PIN);
    GPIOPinTypeTimer(SERVO2_PORT, SERVO2_PIN);
    GPIOPinTypeTimer(SERVO3_PORT, SERVO3_PIN);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
    TimerControlLevel(TIMER0_BASE, TIMER_BOTH, true);

    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
    TimerControlLevel(TIMER1_BASE, TIMER_BOTH, true);

    TimerPrescaleSet(TIMER0_BASE, TIMER_BOTH, ext);
    TimerLoadSet(TIMER0_BASE, TIMER_BOTH, period);

    TimerPrescaleSet(TIMER1_BASE, TIMER_BOTH, ext);
    TimerLoadSet(TIMER1_BASE, TIMER_BOTH, period);
}

//-----------------------------------------------------------------

void servoInit()
{
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    TimerEnable(TIMER1_BASE, TIMER_BOTH);
}

//-----------------------------------------------------------------

void servoSetPulse(uint32_t base, uint32_t timer, uint16_t pulse)
{
    uint32_t period = cycle20ms * pulse / 20000;
    uint8_t ext = period >> 16;
    period &= 0xFFFF;

    TimerMatchSet(base, timer, period);
    TimerPrescaleMatchSet(base, timer, ext);
}

