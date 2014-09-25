/*
 * tickstamp.h
 *
 *  Created on: 25 wrz 2014
 *      Author: Korzo
 */

#ifndef TICKSTAMP_H_
#define TICKSTAMP_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------

uint32_t tickstamp_get_systick(void);

bool tickstamp_is_older(uint32_t time, uint32_t threshold);

//-----------------------------------------------------------------

#endif /* TICKSTAMP_H_ */
