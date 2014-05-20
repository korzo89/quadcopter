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

void led_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeGPIOOutput(LED_GPIO, LED0_PIN | LED1_PIN | LED2_PIN);
    GPIOPinWrite(LED_GPIO, LED0_PIN | LED1_PIN | LED2_PIN, 0x00);
}

//-----------------------------------------------------------------

void led_turn_on(unsigned char leds)
{
    GPIOPinWrite(LED_GPIO, leds & LED_MASK, 0xFF);
}

//-----------------------------------------------------------------

void led_turn_off(unsigned char leds)
{
    GPIOPinWrite(LED_GPIO, leds & LED_MASK, 0x00);
}

//-----------------------------------------------------------------

void led_toggle(unsigned char leds, bool on)
{
    GPIOPinWrite(LED_GPIO, leds & LED_MASK, on ? 0xFF : 0x00);
}

//-----------------------------------------------------------------

void led_set(unsigned char leds)
{
    GPIOPinWrite(LED_GPIO, LED0_PIN | LED1_PIN | LED2_PIN, leds);
}
