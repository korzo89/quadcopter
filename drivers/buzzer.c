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

struct buzzer_obj
{
    xSemaphoreHandle    mutex;
    xTimerHandle        timer;
    uint32_t            curr_step;
    uint32_t            curr_loop;
    const struct buzzer_step *curr_sequence;
};

static struct buzzer_obj buzzer;

//-----------------------------------------------------------------

static void buzzer_process_seq(xTimerHandle tim);

static void buzzer_lock(void);
static void buzzer_unlock(void);

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

    buzzer.mutex = xSemaphoreCreateRecursiveMutex();

    buzzer.timer = xTimerCreate((const signed char*)"buz_tim", MSEC_TO_TICKS(1000), pdFALSE,
            BUZZER_SEQ_TIMER_ID, buzzer_process_seq);
}

//-----------------------------------------------------------------

void buzzer_set_freq(uint32_t freq)
{
    buzzer_lock();

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

    buzzer_unlock();
}

//-----------------------------------------------------------------

result_t buzzer_play_seq(const struct buzzer_step *seq)
{
    buzzer_lock();

    xTimerStop(buzzer.timer, 0);

    buzzer.curr_sequence = seq;
    buzzer.curr_step = 0;
    buzzer.curr_loop = 0;

    buzzer_process_seq(buzzer.timer);

    buzzer_unlock();
    return RES_OK;
}

//-----------------------------------------------------------------

result_t buzzer_stop_seq(void)
{
    buzzer_lock();

    xTimerStop(buzzer.timer, 0);
    buzzer_set_freq(0);

    buzzer_unlock();
    return RES_OK;
}

//-----------------------------------------------------------------

static void buzzer_process_seq(xTimerHandle tim)
{
    (void)tim;

    buzzer_lock();

    const struct buzzer_step *step = &buzzer.curr_sequence[buzzer.curr_step];

    switch (step->action)
    {
    case SEQ_STOP:
        buzzer_set_freq(0);
        return;
    case SEQ_LOOP:
        if (++buzzer.curr_loop == step->freq && step->freq > 0)
        {
            buzzer_set_freq(0);
            return;
        }
        buzzer.curr_step = 0;
        step = &buzzer.curr_sequence[0];
        break;
    default:
        break;
    }

    buzzer_set_freq(step->freq);
    buzzer.curr_step++;

    xTimerChangePeriod(buzzer.timer, MSEC_TO_TICKS(step->action), 0);
    xTimerStart(buzzer.timer, 0);

    buzzer_unlock();
}

//-----------------------------------------------------------------

static void buzzer_lock(void)
{
    xSemaphoreTakeRecursive(buzzer.mutex, portMAX_DELAY);
}

//-----------------------------------------------------------------

static void buzzer_unlock(void)
{
    xSemaphoreGiveRecursive(buzzer.mutex);
}
