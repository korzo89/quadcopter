/*
 * watchdog.c
 *
 *  Created on: 12 cze 2014
 *      Author: Korzo
 */

#include "watchdog.h"
#include <stellaris_config.h>

//-----------------------------------------------------------------

void watchdog_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
}

//-----------------------------------------------------------------

void watchdog_enable(void)
{
    WatchdogResetEnable(WATCHDOG0_BASE);
    WatchdogEnable(WATCHDOG0_BASE);
}

//-----------------------------------------------------------------

void watchdog_disable(void)
{
    WatchdogResetDisable(WATCHDOG0_BASE);
}

//-----------------------------------------------------------------

void watchdog_reload_set(uint32_t val)
{
    WatchdogReloadSet(WATCHDOG0_BASE, val);
}

//-----------------------------------------------------------------

void watchdog_clear(void)
{
    WatchdogIntClear(WATCHDOG0_BASE);
}
