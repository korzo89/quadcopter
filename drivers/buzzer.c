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
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <utils/delay.h>

//-----------------------------------------------------------------

#define BUZZER_TIMER            WTIMER1_BASE
#define BUZZER_SEQ_TIMER_ID     0

//-----------------------------------------------------------------

static xSemaphoreHandle mutex;
static xTimerHandle timer;
static buzzer_step_t *curr_sequence;
static unsigned int curr_step;

//-----------------------------------------------------------------

static void buzzer_process_seq(xTimerHandle);

//-----------------------------------------------------------------

void buzzer_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF);

    GPIOPinConfigure(GPIO_PC6_WT1CCP0);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6);

    TimerConfigure(BUZZER_TIMER, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
    TimerControlLevel(BUZZER_TIMER, TIMER_A, true);

    buzzer_set_freq(10);
    buzzer_set_freq(0);

    mutex = xSemaphoreCreateMutex();

    timer = xTimerCreate((const signed char*)"buz_tim", MSEC_TO_TICKS(1000), pdFALSE,
            BUZZER_SEQ_TIMER_ID, buzzer_process_seq);
}

//-----------------------------------------------------------------

void buzzer_set_freq(uint32_t freq)
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

//-----------------------------------------------------------------

result_t buzzer_play_seq(buzzer_step_t *seq)
{
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
        return RES_ERR_FATAL;

    xTimerStop(timer, 0);

    curr_sequence = seq;
    curr_step = 0;

    buzzer_process_seq(timer);

    xSemaphoreGive(mutex);
    return RES_OK;
}

//-----------------------------------------------------------------

result_t buzzer_stop_seq(void)
{
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
        return RES_ERR_FATAL;

    xTimerStop(timer, 0);
    buzzer_set_freq(0);

    xSemaphoreGive(mutex);
    return RES_OK;
}

//-----------------------------------------------------------------

static void buzzer_process_seq(xTimerHandle tim)
{
    (void)tim;

    buzzer_step_t step = curr_sequence[curr_step];

    switch (step.action)
    {
    case BUZZER_SEQ_STOP:
        buzzer_set_freq(0);
        return;
    case BUZZER_SEQ_LOOP:
        curr_step = 0;
        step = curr_sequence[0];
        break;
    default:
        break;
    }

    buzzer_set_freq(step.freq);
    curr_step++;

    xTimerChangePeriod(timer, MSEC_TO_TICKS(step.action), 0);
    xTimerStart(timer, 0);
}
