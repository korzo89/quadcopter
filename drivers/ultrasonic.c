/*
 * ultrasonic.c
 *
 *  Created on: 07-05-2014
 *      Author: Korzo
 */

#include "ultrasonic.h"

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/timer.h>
#include <stdint.h>

//-----------------------------------------------------------------

#define US_TIMER                WTIMER2_BASE
#define US_PRESCALE_1_80        79
#define US_TRIG_PULSE           10

#define US_MAX_DIST             200
#define US_TIMER_MAX            ( US_MAX_DIST * 58 )

#define US_TRIG_GPIO            GPIO_PORTD_BASE
#define US_TRIG_PIN             GPIO_PIN_2

#define US_ECHO_GPIO            GPIO_PORTD_BASE
#define US_ECHO_PIN             GPIO_PIN_3

#define TRIG_SET()              GPIOPinWrite(US_TRIG_GPIO, US_TRIG_PIN, 0xFF);
#define TRIG_CLEAR()            GPIOPinWrite(US_TRIG_GPIO, US_TRIG_PIN, 0x00);

#define ECHO_STATUS()           ( GPIOPinRead(US_ECHO_GPIO, US_ECHO_PIN) )

#define USEC_TO_CM(x)           ( (float)(x) / 58.0 )

//-----------------------------------------------------------------

void ultrasonicConfig(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinTypeGPIOOutput(US_TRIG_GPIO, US_TRIG_PIN);
    GPIOPinTypeGPIOInput(US_ECHO_GPIO, US_ECHO_PIN);

    TRIG_CLEAR();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);

    TimerConfigure(US_TIMER, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT);
    TimerPrescaleSet(US_TIMER, TIMER_A, US_PRESCALE_1_80);
    TimerIntEnable(US_TIMER, TIMER_TIMA_TIMEOUT);
}

//-----------------------------------------------------------------

float ultrasonicGetDistance(void)
{
    TRIG_CLEAR();

    TimerIntClear(US_TIMER, TIMER_TIMA_TIMEOUT);
    TimerLoadSet(US_TIMER, TIMER_A, US_TIMER_MAX);
    TimerEnable(US_TIMER, TIMER_A);
    // wait for ECHO high or timeout
    while (ECHO_STATUS() && !TimerIntStatus(US_TIMER, TIMER_TIMA_TIMEOUT));
    TimerIntClear(US_TIMER, TIMER_TIMA_TIMEOUT);
    TimerDisable(US_TIMER, TIMER_A);

    // prepare trigger timer
    TimerLoadSet(US_TIMER, TIMER_A, US_TRIG_PULSE);
    // send trigger pulse
    TRIG_SET();
    TimerEnable(US_TIMER, TIMER_A);
    while (!TimerIntStatus(US_TIMER, TIMER_TIMA_TIMEOUT));
    TimerIntClear(US_TIMER, TIMER_TIMA_TIMEOUT);
    TRIG_CLEAR();

    TimerLoadSet(US_TIMER, TIMER_A, US_TIMER_MAX);
    TimerEnable(US_TIMER, TIMER_A);
    // wait for ECHO high or timeout
    while (!ECHO_STATUS() && !TimerIntStatus(US_TIMER, TIMER_TIMA_TIMEOUT));
    TimerIntClear(US_TIMER, TIMER_TIMA_TIMEOUT);
    TimerDisable(US_TIMER, TIMER_A);

    TimerLoadSet(US_TIMER, TIMER_A, US_TIMER_MAX);
    TimerEnable(US_TIMER, TIMER_A);
    // wait until ECHO low or timeout
    while (ECHO_STATUS() && !TimerIntStatus(US_TIMER, TIMER_TIMA_TIMEOUT));
    TimerDisable(US_TIMER, TIMER_A);

    uint32_t count;
    if (TimerIntStatus(US_TIMER, TIMER_TIMA_TIMEOUT))
    {
        TimerIntClear(US_TIMER, TIMER_TIMA_TIMEOUT);
        count = US_TIMER_MAX;
    }
    else
    {
        count = US_TIMER_MAX - TimerValueGet(US_TIMER, TIMER_A);
    }

    return USEC_TO_CM(count);
}
