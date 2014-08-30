/*
 * adc.h
 *
 *  Created on: 08-05-2014
 *      Author: Korzo
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

//-----------------------------------------------------------------

void adc_init(void);

uint32_t adc_get_value(void);

//-----------------------------------------------------------------

#endif /* ADC_H_ */
