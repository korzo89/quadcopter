/*
 * led.h
 *
 *  Created on: 03-11-2013
 *      Author: Korzo
 */

#ifndef LED_H_
#define LED_H_

//-----------------------------------------------------------------

#include <defs.h>
#include <stellaris_config.h>

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

void led_init(void);

void led_turn_on(uint8_t leds);
void led_turn_off(uint8_t leds);
void led_toggle(uint8_t leds, bool on);
void led_set(uint8_t leds);

//-----------------------------------------------------------------

#endif /* LED_H_ */
