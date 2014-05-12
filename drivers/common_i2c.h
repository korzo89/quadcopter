#ifndef COMMON_I2C_H_
#define COMMON_I2C_H_

//-----------------------------------------------------------------

#include <stdbool.h>
#include <drivers/i2c.h>
#include <inc/hw_memmap.h>

//-----------------------------------------------------------------

#define COMMON_I2C_BASE         I2C2_MASTER_BASE

#define COMMON_I2C_ERR_BLOCKED  0xFFF

//-----------------------------------------------------------------

void comI2CInit(void);

unsigned long comI2CWriteRegister(uint8_t addr, uint8_t reg, uint8_t data);
unsigned long comI2CWriteRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len);

uint8_t comI2CReadRegister(uint8_t addr, uint8_t reg, unsigned long *res);
unsigned long comI2CReadRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len);

bool comI2CLock(void);
void comI2CUnlock(void);

//-----------------------------------------------------------------

#endif /* COMMON_I2C_H_ */

