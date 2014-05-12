/*
 * led.c
 *
 *  Created on: 03-11-2013
 *      Author: Korzo
 */

#include "led.h"

#include <utils/delay.h>
#include <driverlib/sysctl.h>

//-----------------------------------------------------------------

#define LED_MASK        (LED0_PIN | LED1_PIN | LED2_PIN)

//-----------------------------------------------------------------

void ledInit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeGPIOOutput(LED_GPIO, LED0_PIN | LED1_PIN | LED2_PIN);
    GPIOPinWrite(LED_GPIO, LED0_PIN | LED1_PIN | LED2_PIN, 0x00);
}

//-----------------------------------------------------------------

void ledTurnOn(unsigned char leds)
{
    GPIOPinWrite(LED_GPIO, leds & LED_MASK, 0xFF);
}

//-----------------------------------------------------------------

void ledTurnOff(unsigned char leds)
{
    GPIOPinWrite(LED_GPIO, leds & LED_MASK, 0x00);
}

//-----------------------------------------------------------------

void ledToggle(unsigned char leds, bool on)
{
    GPIOPinWrite(LED_GPIO, leds & LED_MASK, on ? 0xFF : 0x00);
}

//-----------------------------------------------------------------

void ledSet(unsigned char leds)
{
    GPIOPinWrite(LED_GPIO, LED0_PIN | LED1_PIN | LED2_PIN, leds);
}
