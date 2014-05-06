/*
 * delay.c
 *
 *  Created on: 25-09-2013
 *      Author: Korzo
 */

//-----------------------------------------------------------------

#include "delay.h"

#include <FreeRTOS.h>
#include <task.h>

//-----------------------------------------------------------------

void delay(unsigned long ms)
{
    vTaskDelay(MSEC_TO_TICKS(ms));
}

