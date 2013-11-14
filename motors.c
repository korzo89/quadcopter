/*
 * servo.c
 *
 *  Created on: 31-08-2013
 *      Author: Korzo
 */

#include "motors.h"
#include "led.h"

#include <stdbool.h>
#include <math.h>

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

//-----------------------------------------------------------------

#define min(a,b)                (((a) < (b)) ? (a) : (b))
#define max(a,b)                (((a) > (b)) ? (a) : (b))

#define THROTTLE_TO_PULSE(x)    (1000 + (uint16_t) min(max(0, (x)), THROTTLE_MAX))

//-----------------------------------------------------------------

static uint32_t cycle20ms;

static bool armed = false;

//-----------------------------------------------------------------

void motorsConfig()
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
}

//-----------------------------------------------------------------

void motorsInit()
{
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    TimerEnable(TIMER1_BASE, TIMER_BOTH);

    motorsDisarm();
}

//-----------------------------------------------------------------

bool motorsArmed(void)
{
    return armed;
}

//-----------------------------------------------------------------

void motorsArm(void)
{
    armed = true;

    LEDTurnOff(LED_GREEN);
    LEDTurnOn(LED_RED);
}

//-----------------------------------------------------------------

void motorsDisarm(void)
{
    armed = false;
    motorsSetThrottle(0.0f, 0.0f, 0.0f, 0.0f);

    LEDTurnOn(LED_GREEN);
    LEDTurnOff(LED_RED);
}

//-----------------------------------------------------------------

void motorsSetThrottle(float m1, float m2, float m3, float m4)
{
    motorsServoPulse(MOTOR1_BASE, MOTOR1_TIMER, THROTTLE_TO_PULSE(m1));
    motorsServoPulse(MOTOR2_BASE, MOTOR2_TIMER, THROTTLE_TO_PULSE(m2));
    motorsServoPulse(MOTOR3_BASE, MOTOR3_TIMER, THROTTLE_TO_PULSE(m3));
    motorsServoPulse(MOTOR4_BASE, MOTOR4_TIMER, THROTTLE_TO_PULSE(m4));
}

//-----------------------------------------------------------------

void motorsServoPulse(uint32_t base, uint32_t timer, uint16_t pulse)
{
    uint32_t period = cycle20ms * pulse / 20000;
    uint8_t ext = period >> 16;
    period &= 0xFFFF;

    TimerMatchSet(base, timer, period);
    TimerPrescaleMatchSet(base, timer, ext);
}

