#ifndef IMU_I2C_H_
#define IMU_I2C_H_

//-----------------------------------------------------------------

#include <stdbool.h>
#include <drivers/i2c.h>
#include <inc/hw_memmap.h>

//-----------------------------------------------------------------

#define IMU_I2C_BASE         I2C0_MASTER_BASE

#define IMU_I2C_ERR_BLOCKED  0xFFF

//-----------------------------------------------------------------

void imuI2CInit(void);

unsigned long imuI2CWriteRegister(uint8_t addr, uint8_t reg, uint8_t data);
unsigned long imuI2CWriteRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len);

uint8_t imuI2CReadRegister(uint8_t addr, uint8_t reg, unsigned long *res);
unsigned long imuI2CReadRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len);

bool imuI2CLock(void);
void imuI2CUnlock(void);

//-----------------------------------------------------------------

#endif /* IMU_I2C_H_ */

