/*
 * hmc5883.c
 *
 *  Created on: 24-09-2013
 *      Author: Korzo
 */

#include "hmc5883.h"
#include <drivers/imu_i2c.h>

//-----------------------------------------------------------------

static uint8_t buffer[6];

//-----------------------------------------------------------------

void hmc5883Init()
{
    imuI2CWriteRegister(HMC5883_I2C_ADDR, HMC5883_MODE, 0x00);
}

//-----------------------------------------------------------------

void hmc5883ReadMag(int16_t *x, int16_t *y, int16_t *z)
{
    imuI2CReadRegisterBurst(HMC5883_I2C_ADDR, HMC5883_OUT_X_MSB, buffer, 6);

    *x = ((int16_t) buffer[0] << 8) | buffer[1];
    *z = ((int16_t) buffer[2] << 8) | buffer[3];
    *y = ((int16_t) buffer[4] << 8) | buffer[5];
}

