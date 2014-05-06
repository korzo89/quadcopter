/*
 * utils.c
 *
 *  Created on: 25-09-2013
 *      Author: Korzo
 */

//-----------------------------------------------------------------

#include "utils.h"

#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

//-----------------------------------------------------------------

//extern volatile unsigned long sysTickCount;

//-----------------------------------------------------------------

void delay(unsigned long ms)
{
//    sysTickCount = ms;
//    while (sysTickCount);
    vTaskDelay(ms / portTICK_RATE_MS);
}

