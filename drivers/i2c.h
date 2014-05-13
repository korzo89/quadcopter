/*
 * i2c.h
 *
 *  Created on: 22-09-2013
 *      Author: Korzo
 */

#ifndef I2C_H_
#define I2C_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <inc/hw_memmap.h>

//-----------------------------------------------------------------

unsigned long i2cWriteRegister(unsigned long base, uint8_t addr, uint8_t reg, uint8_t data);
unsigned long i2cWriteRegisterBurst(unsigned long base, uint8_t addr, uint8_t reg, uint8_t *buf, int len);

uint8_t i2cReadRegister(unsigned long base, uint8_t addr, uint8_t reg, unsigned long *res);
unsigned long i2cReadRegisterBurst(unsigned long base, uint8_t addr, uint8_t reg, uint8_t *buf, int len);

unsigned long i2cDataPut(unsigned long base, uint8_t data, unsigned long control);

//-----------------------------------------------------------------

#endif /* I2C_H_ */
