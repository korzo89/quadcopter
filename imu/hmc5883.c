/*
 * hmc5883.c
 *
 *  Created on: 24-09-2013
 *      Author: Korzo
 */

#include "hmc5883.h"
#include <i2c.h>

//-----------------------------------------------------------------

#define HMC8553_I2C_BASE        I2C0_MASTER_BASE

//-----------------------------------------------------------------

static uint8_t buffer[6];

//-----------------------------------------------------------------

void HMC5883_init()
{
    I2CWriteRegister(HMC8553_I2C_BASE, HMC5883_I2C_ADDR, HMC5883_MODE, 0x00);
}

//-----------------------------------------------------------------

void HMC5883_readMag(int16_t *x, int16_t *y, int16_t *z)
{
    I2CReadRegisterBurst(HMC8553_I2C_BASE, HMC5883_I2C_ADDR, HMC5883_OUT_X_MSB, buffer, 6);

    *x = ((int16_t) buffer[0] << 8) | buffer[1];
    *z = ((int16_t) buffer[2] << 8) | buffer[3];
    *y = ((int16_t) buffer[4] << 8) | buffer[5];
}

