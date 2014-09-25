/*
 * tickstamp.c
 *
 *  Created on: 25 wrz 2014
 *      Author: Korzo
 */

#include "tickstamp.h"
#include <FreeRTOS.h>
#include <task.h>

//-----------------------------------------------------------------

uint32_t tickstamp_get_systick(void)
{
    return xTaskGetTickCount();
}

//-----------------------------------------------------------------

bool tickstamp_is_older(uint32_t time, uint32_t threshold)
{
    uint32_t now = xTaskGetTickCount();
    return (now - time) > threshold;
}
