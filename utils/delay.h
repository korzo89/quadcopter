/*
 * delay.h
 *
 *  Created on: 25-09-2013
 *      Author: Korzo
 */

#ifndef DELAY_H_
#define DELAY_H_

//-----------------------------------------------------------------

#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>

//-----------------------------------------------------------------

#define MSEC_TO_TICKS(x)        ( (x) / portTICK_RATE_MS )
#define DELAY_MS(x)             vTaskDelay(MSEC_TO_TICKS(x))

//-----------------------------------------------------------------

#endif /* DELAY_H_ */
