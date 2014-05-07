/*
 * l3g4200d.c
 *
 *  Created on: 24-09-2013
 *      Author: Korzo
 */

#include "l3g4200d.h"
#include <drivers/i2c.h>
#include <stdlib.h>

//-----------------------------------------------------------------

#define L3G4200_I2C_BASE        I2C0_MASTER_BASE

//-----------------------------------------------------------------

static uint8_t buffer[6];

//-----------------------------------------------------------------

void L3G4200D_init()
{
    L3G4200D_writeRegister(L3G4200D_CTRL_REG4, 0xA0);
    L3G4200D_writeRegister(L3G4200D_CTRL_REG1, 0x4F);
}

//-----------------------------------------------------------------

void L3G4200D_readGyro(int16_t *x, int16_t *y, int16_t *z)
{
    L3G4200D_readRegisterBurst(L3G4200D_OUT_X_L, buffer, 6);

    *x = ((int16_t) buffer[1] << 8) | buffer[0];
    *y = ((int16_t) buffer[3] << 8) | buffer[2];
    *z = ((int16_t) buffer[5] << 8) | buffer[4];
}

//-----------------------------------------------------------------

void L3G4200D_writeRegister(uint8_t reg, uint8_t data)
{
    I2CWriteRegister(L3G4200_I2C_BASE, L3G4200D_I2C_ADDR, reg, data);
}

//-----------------------------------------------------------------

uint8_t L3G4200D_readRegister(uint8_t reg)
{
    return I2CReadRegister(L3G4200_I2C_BASE, L3G4200D_I2C_ADDR, reg, NULL);
}

//-----------------------------------------------------------------

void L3G4200D_readRegisterBurst(uint8_t reg, uint8_t *buf, int len)
{
    I2CReadRegisterBurst(L3G4200_I2C_BASE, L3G4200D_I2C_ADDR, reg | 0x80, buf, len);
}
