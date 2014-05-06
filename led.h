/*
 * led.h
 *
 *  Created on: 03-11-2013
 *      Author: Korzo
 */

#ifndef LED_H_
#define LED_H_

//-----------------------------------------------------------------

#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

//-----------------------------------------------------------------

#define LED_GPIO        GPIO_PORTE_BASE
#define LED0_PIN        GPIO_PIN_3
#define LED1_PIN        GPIO_PIN_2
#define LED2_PIN        GPIO_PIN_1

#define LED_RED         LED2_PIN
#define LED_YELLOW      LED1_PIN
#define LED_GREEN       LED0_PIN

#define LEDS_OFF        0x00

//-----------------------------------------------------------------

void LEDConfig();

void LEDTurnOn(unsigned char leds);
void LEDTurnOff(unsigned char leds);
void LEDToggle(unsigned char leds, bool on);
void LEDSet(unsigned char leds);

//-----------------------------------------------------------------

#endif /* LED_H_ */
