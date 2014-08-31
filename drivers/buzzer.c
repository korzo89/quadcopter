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
#include <utils/fifo.h>
#include <string.h>

//-----------------------------------------------------------------

#define BUZZER_TIMER            WTIMER1_BASE
#define BUZZER_SEQ_TIMER_ID     0

#define BUZZER_QUEUE_MAX        8

//-----------------------------------------------------------------

struct buzzer_obj
{
    xSemaphoreHandle    mutex;
    xTimerHandle        timer;

    uint32_t            curr_step;
    uint32_t            curr_loop;
    const struct buzzer_step *curr_sequence;
    bool                playing;

    struct fifo         queue;
    const struct buzzer_step *queue_buf[BUZZER_QUEUE_MAX];
};

static struct buzzer_obj buzzer;

//-----------------------------------------------------------------

static void process_seq(xTimerHandle tim);
static void finish_seq(void);
static void play_seq(const struct buzzer_step *seq);
static void stop_seq(void);

static void buzzer_lock(void);
static void buzzer_unlock(void);

//-----------------------------------------------------------------

void buzzer_init(void)
{
    memset(&buzzer, 0, sizeof(buzzer));

    buzzer.mutex = xSemaphoreCreateRecursiveMutex();
    buzzer.timer = xTimerCreate(TASK_NAME("buz_tim"), MSEC_TO_TICKS(1000), pdFALSE,
            BUZZER_SEQ_TIMER_ID, process_seq);

    fifo_init(&buzzer.queue, buzzer.queue_buf, BUZZER_QUEUE_MAX, sizeof(const struct buzzer_step*));

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

bool buzzer_play_seq(const struct buzzer_step *seq, enum buzzer_mode mode)
{
    if (!seq)
        return false;

    buzzer_lock();

    bool res = false;

    switch (mode)
    {
    case BUZZER_MODE_FORCE:
        stop_seq();
        play_seq(seq);
        res = true;
        break;
    case BUZZER_MODE_QUEUE:
        if (buzzer.playing)
        {
            res = fifo_enqueue(&buzzer.queue, &seq);
        }
        else
        {
            play_seq(seq);
            res = true;
        }
        break;
    default:
        res = true;
        if (!buzzer.playing)
            play_seq(seq);
        break;
    }

    buzzer_unlock();
    return res;
}

//-----------------------------------------------------------------

static void play_seq(const struct buzzer_step *seq)
{
    buzzer.curr_sequence = seq;
    buzzer.curr_step = 0;
    buzzer.curr_loop = 0;
    buzzer.playing = true;

    process_seq(buzzer.timer);
}

//-----------------------------------------------------------------

static void stop_seq(void)
{
    xTimerStop(buzzer.timer, 0);
    buzzer_set_freq(0);
}

//-----------------------------------------------------------------

void buzzer_stop(bool clear_queue)
{
    buzzer_lock();

    stop_seq();

    if (clear_queue)
        fifo_clear(&buzzer.queue);

    buzzer_unlock();
}

//-----------------------------------------------------------------

static void process_seq(xTimerHandle tim)
{
    (void)tim;

    buzzer_lock();

    const struct buzzer_step *step = &buzzer.curr_sequence[buzzer.curr_step];

    switch (step->action)
    {
    case SEQ_STOP:
        finish_seq();
        buzzer_unlock();
        return;
    case SEQ_LOOP:
        if (++buzzer.curr_loop == step->freq && step->freq > 0)
        {
            finish_seq();
            buzzer_unlock();
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

static void finish_seq(void)
{
    buzzer_set_freq(0);
    buzzer.playing = false;

    const struct buzzer_step *next = NULL;
    if (fifo_dequeue(&buzzer.queue, &next))
        play_seq(next);
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
