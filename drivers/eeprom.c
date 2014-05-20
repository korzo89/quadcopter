/*
 * eeprom.c
 *
 *  Created on: 06-05-2014
 *      Author: Korzo
 */

#include "eeprom.h"

//-----------------------------------------------------------------

#include <inc/hw_types.h>
#include <driverlib/i2c.h>

#include <drivers/ext_i2c.h>

//-----------------------------------------------------------------

#define EEPROM_ADDR         0x50

//-----------------------------------------------------------------

static i2c_t *i2c_if;

//-----------------------------------------------------------------

void eeprom_init(void)
{
    i2c_if = ext_i2c_get_if();
}

//-----------------------------------------------------------------

result_t eeprom_write(uint16_t addr, uint8_t *buf, unsigned int len)
{
    return i2c_write_reg16(i2c_if, EEPROM_ADDR, addr, buf, len);
}

//-----------------------------------------------------------------

result_t eeprom_write_byte(uint16_t addr, uint8_t data)
{
    return i2c_write_reg16(i2c_if, EEPROM_ADDR, addr, &data, 1);
}

//-----------------------------------------------------------------

result_t eeprom_read(uint16_t addr, uint8_t *buf, unsigned int len)
{
    return i2c_read_reg16(i2c_if, EEPROM_ADDR, addr, buf, len);
}

//-----------------------------------------------------------------

result_t eeprom_read_byte(uint16_t addr, uint8_t *data)
{
    return i2c_read_reg16(i2c_if, EEPROM_ADDR, addr, data, 1);
}
