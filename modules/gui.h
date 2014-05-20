/*
 * gui.h
 *
 *  Created on: 20 maj 2014
 *      Author: Korzo
 */

#ifndef GUI_H_
#define GUI_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

#define BUTTON_READ()      GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0)
#define BUTTON_PRESSED()   (!BUTTON_READ())

//-----------------------------------------------------------------

result_t gui_init(void);

//-----------------------------------------------------------------

#endif /* GUI_H_ */
