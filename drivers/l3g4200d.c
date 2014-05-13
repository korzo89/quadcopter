/*
 * l3g4200d.c
 *
 *  Created on: 24-09-2013
 *      Author: Korzo
 */

#include "l3g4200d.h"
#include <drivers/imu_i2c.h>

//-----------------------------------------------------------------

static uint8_t buffer[6];

//-----------------------------------------------------------------

void l3g4200dInit()
{
    l3g4200dWriteRegister(L3G4200D_CTRL_REG4, 0xA0);
    l3g4200dWriteRegister(L3G4200D_CTRL_REG1, 0x4F);
}

//-----------------------------------------------------------------

void l3g4200dReadGyro(int16_t *x, int16_t *y, int16_t *z)
{
    l3g4200dReadRegisterBurst(L3G4200D_OUT_X_L, buffer, 6);

    *x = ((int16_t) buffer[1] << 8) | buffer[0];
    *y = ((int16_t) buffer[3] << 8) | buffer[2];
    *z = ((int16_t) buffer[5] << 8) | buffer[4];
}

//-----------------------------------------------------------------

void l3g4200dWriteRegister(uint8_t reg, uint8_t data)
{
    imuI2CWriteRegister(L3G4200D_I2C_ADDR, reg, data);
}

//-----------------------------------------------------------------

uint8_t l3g4200dReadRegister(uint8_t reg)
{
    return imuI2CReadRegister(L3G4200D_I2C_ADDR, reg, NULL);
}

//-----------------------------------------------------------------

void l3g4200dReadRegisterBurst(uint8_t reg, uint8_t *buf, int len)
{
    imuI2CReadRegisterBurst(L3G4200D_I2C_ADDR, reg | 0x80, buf, len);
}
