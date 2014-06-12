/*
 * watchdog.h
 *
 *  Created on: 12 cze 2014
 *      Author: Korzo
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

void watchdog_init(void);

void watchdog_enable(void);
void watchdog_disable(void);

void watchdog_reload_set(uint32_t val);

void watchdog_clear(void);

//-----------------------------------------------------------------

#endif /* WATCHDOG_H_ */
