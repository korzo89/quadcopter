/*
 * eeprom.h
 *
 *  Created on: 06-05-2014
 *      Author: Korzo
 */

#ifndef EEPROM_H_
#define EEPROM_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------

bool eepromWrite(uint16_t addr, uint8_t *buf, unsigned int len);
bool eepromWriteByte(uint16_t addr, uint8_t data);

bool eepromRead(uint16_t addr, uint8_t *buf, unsigned int len);
uint8_t eepromReadByte(uint16_t addr, bool *res);

//-----------------------------------------------------------------

#endif /* EEPROM_H_ */
