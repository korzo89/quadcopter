/*
 * FreeRTOS_hooks.c
 *
 *  Created on: 08-05-2014
 *      Author: Korzo
 */

#include <FreeRTOS.h>
#include <task.h>

//-----------------------------------------------------------------

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}
