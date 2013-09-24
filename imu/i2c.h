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

#define I2C_BASE I2C0_MASTER_BASE

//-----------------------------------------------------------------

void I2CWriteRegister(uint8_t addr, uint8_t reg, uint8_t data);
void I2CWriteRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len);

uint8_t I2CReadRegister(uint8_t addr, uint8_t reg);
void I2CReadRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len);


//-----------------------------------------------------------------

#endif /* I2C_H_ */
