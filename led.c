/*
 * led.c
 *
 *  Created on: 03-11-2013
 *      Author: Korzo
 */

#include "led.h"
#include <utils/delay.h>

#include "driverlib/sysctl.h"

//-----------------------------------------------------------------

#define LED_MASK        (LED0_PIN | LED1_PIN | LED2_PIN)

//-----------------------------------------------------------------

void LEDConfig()
{
    GPIOPinTypeGPIOOutput(LED_GPIO, LED0_PIN | LED1_PIN | LED2_PIN);
    GPIOPinWrite(LED_GPIO, LED0_PIN | LED1_PIN | LED2_PIN, 0x00);
}

//-----------------------------------------------------------------

void LEDTurnOn(unsigned char leds)
{
    GPIOPinWrite(LED_GPIO, leds & LED_MASK, 0xFF);
}

//-----------------------------------------------------------------

void LEDTurnOff(unsigned char leds)
{
    GPIOPinWrite(LED_GPIO, leds & LED_MASK, 0x00);
}

//-----------------------------------------------------------------

void LEDToggle(unsigned char leds, bool on)
{
    GPIOPinWrite(LED_GPIO, leds & LED_MASK, on ? 0xFF : 0x00);
}

//-----------------------------------------------------------------

void LEDSet(unsigned char leds)
{
    GPIOPinWrite(LED_GPIO, LED0_PIN | LED1_PIN | LED2_PIN, leds);
}
