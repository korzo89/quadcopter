/*
 * eeprom.h
 *
 *  Created on: 06-05-2014
 *      Author: Korzo
 */

#ifndef EEPROM_H_
#define EEPROM_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

void eeprom_init(void);

result_t eeprom_write(uint16_t addr, uint8_t *buf, unsigned int len);
result_t eeprom_write_byte(uint16_t addr, uint8_t data);

result_t eeprom_read(uint16_t addr, uint8_t *buf, unsigned int len);
result_t eeprom_read_byte(uint16_t addr, uint8_t *data);

bool eeprom_is_ready(void);

//-----------------------------------------------------------------

#endif /* EEPROM_H_ */
