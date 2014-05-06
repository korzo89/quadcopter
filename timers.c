/*
 * timers.c
 *
 *  Created on: 18-10-2013
 *      Author: Korzo
 */

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"

#include "pid.h"
#include "imu/imu.h"
#include "motors.h"
#include "led.h"
#include "drivers/oled.h"

#include <utils/ustdlib.h>

//-----------------------------------------------------------------

volatile unsigned long sysTickCount = 0;

//-----------------------------------------------------------------

extern volatile int lostCount;

extern PID_t pitchPID;

//-----------------------------------------------------------------

void timersConfig(void)
{
    TimerConfigure(TIMER2_BASE, TIMER_CFG_32_BIT_PER);
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet() / 100);

    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER2A);

    TimerEnable(TIMER2_BASE, TIMER_A);
}

//-----------------------------------------------------------------

void SysTickISR(void)
{
    if (sysTickCount)
        sysTickCount--;
}

//-----------------------------------------------------------------

void Timer2AIntHandler(void)
{
    float pitch, roll, yaw;
    static int count = 0;
    static char buf[30];

    if (!TimerIntStatus(TIMER2_BASE, TIMER_TIMA_TIMEOUT))
        return;

    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    count++;
    LEDToggle(LED_YELLOW, GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0) && count < 25);
    if (count == 50)
        count = 0;

    lostCount++;
    if (lostCount == 300)
        motorsDisarm();

    IMUPollSensors();
    IMUUpdate();

    IMUGetEulerAngles(&pitch, &roll, &yaw);
//    UARTprintf("%d %d %d\r\n", (int)pitch, (int)roll, (int)yaw);

    PIDUpdate(&pitchPID, pitch, 0.01f);

    if (count == 49)
    {
        usprintf(buf, "Pitch: %4d", (int)pitch);
        oledDispStrAt(buf, 1, 0);
        usprintf(buf, "Roll:  %4d", (int)roll);
        oledDispStrAt(buf, 2, 0);
        usprintf(buf, "Yaw:   %4d", (int)yaw);
        oledDispStrAt(buf, 3, 0);

        if (motorsArmed())
            oledDispStrAt(" *** ARMED! *** ", 7, 0);
        else
            oledDispStrAt("                ", 7, 0);
    }
}
