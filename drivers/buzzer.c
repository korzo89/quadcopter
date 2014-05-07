/*
 * buzzer.c
 *
 *  Created on: 07-05-2014
 *      Author: Korzo
 */

#include "buzzer.h"

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/timer.h>
#include <stdint.h>

//-----------------------------------------------------------------

#define BUZZER_TIMER       WTIMER1_BASE

//-----------------------------------------------------------------

void buzzerConfig(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF);

    GPIOPinConfigure(GPIO_PC6_WT1CCP0);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6);

    TimerConfigure(BUZZER_TIMER, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
    TimerControlLevel(BUZZER_TIMER, TIMER_A, true);

    buzzerSetFreq(0);
}

//-----------------------------------------------------------------

void buzzerSetFreq(unsigned int freq)
{
    uint32_t cycle = SysCtlClockGet() / freq;
    uint32_t period = cycle;

    TimerPrescaleSet(BUZZER_TIMER, TIMER_A, 0);
    TimerLoadSet(BUZZER_TIMER, TIMER_A, period);

    period = cycle / 2;

    TimerMatchSet(BUZZER_TIMER, TIMER_A, period);
    TimerPrescaleMatchSet(BUZZER_TIMER, TIMER_A, 0);

    if (freq != 0)
        TimerEnable(BUZZER_TIMER, TIMER_A);
    else
        TimerDisable(BUZZER_TIMER, TIMER_A);
}


