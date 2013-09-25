/*
 * utils.c
 *
 *  Created on: 25-09-2013
 *      Author: Korzo
 */

//-----------------------------------------------------------------

#include "utils.h"

//-----------------------------------------------------------------

extern volatile unsigned long sysTickCount;

//-----------------------------------------------------------------

void delay(unsigned long us)
{
    sysTickCount = us;
    while (sysTickCount);
}

