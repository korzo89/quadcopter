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

unsigned long I2CWriteRegister(unsigned long base, uint8_t addr, uint8_t reg, uint8_t data);
unsigned long I2CWriteRegisterBurst(unsigned long base, uint8_t addr, uint8_t reg, uint8_t *buf, int len);

uint8_t I2CReadRegister(unsigned long base, uint8_t addr, uint8_t reg, unsigned long *res);
unsigned long I2CReadRegisterBurst(unsigned long base, uint8_t addr, uint8_t reg, uint8_t *buf, int len);

unsigned long I2CDataPut(unsigned long base, uint8_t data, unsigned long control);

//-----------------------------------------------------------------

#endif /* I2C_H_ */
