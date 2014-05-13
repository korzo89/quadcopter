/*
 * adxl345.c
 *
 *  Created on: 23-09-2013
 *      Author: Korzo
 */

//-----------------------------------------------------------------

#include "adxl345.h"
#include <drivers/imu_i2c.h>

//-----------------------------------------------------------------

static uint8_t buffer[6];

//-----------------------------------------------------------------

void adxl345Init()
{
    imuI2CWriteRegister(ADXL345_I2C_ADDR, ADXL345_POWER_CTL, 0x00);
    imuI2CWriteRegister(ADXL345_I2C_ADDR, ADXL345_POWER_CTL, ADXL345_MEASURE);
}

//-----------------------------------------------------------------

void adxl345GetAcceleration(int16_t *x, int16_t *y, int16_t *z)
{
    imuI2CReadRegisterBurst(ADXL345_I2C_ADDR, ADXL345_DATAX0, buffer, 6);

    *x = ((int16_t)buffer[1] << 8) | buffer[0];
    *y = ((int16_t)buffer[3] << 8) | buffer[2];
    *z = ((int16_t)buffer[5] << 8) | buffer[4];
}

//-----------------------------------------------------------------

ADXL345Range adxl345GetRange()
{
    return (ADXL345Range)(imuI2CReadRegister(ADXL345_I2C_ADDR,
            ADXL345_DATA_FORMAT, NULL) & ADXL345_RANGE_MASK);
}

//-----------------------------------------------------------------

void adxl345SetRange(ADXL345Range range)
{
    uint8_t reg = imuI2CReadRegister(ADXL345_I2C_ADDR, ADXL345_POWER_CTL, NULL);
    reg = (reg & ~ADXL345_RANGE_MASK) | (uint8_t)range;

    imuI2CWriteRegister(ADXL345_I2C_ADDR, ADXL345_POWER_CTL, reg);
}

//-----------------------------------------------------------------

void adxl345GetOffsets(int8_t *x, int8_t *y, int8_t *z)
{
    imuI2CReadRegisterBurst(ADXL345_I2C_ADDR, ADXL345_OFSX, buffer, 3);

    *x = (int8_t)buffer[0];
    *y = (int8_t)buffer[1];
    *z = (int8_t)buffer[2];
}

//-----------------------------------------------------------------

void adxl345SetOffsets(int8_t x, int8_t y, int8_t z)
{
    imuI2CWriteRegister(ADXL345_I2C_ADDR, ADXL345_OFSX, x);
    imuI2CWriteRegister(ADXL345_I2C_ADDR, ADXL345_OFSY, y);
    imuI2CWriteRegister(ADXL345_I2C_ADDR, ADXL345_OFSZ, z);
}

//-----------------------------------------------------------------

uint8_t adxl345GetIntSource()
{
    return imuI2CReadRegister(ADXL345_I2C_ADDR, ADXL345_INT_SOURCE, NULL);
}

//-----------------------------------------------------------------

uint8_t adxl345GetIntMapping()
{
    return imuI2CReadRegister(ADXL345_I2C_ADDR, ADXL345_INT_MAP, NULL);
}

//-----------------------------------------------------------------

void adxl345SetIntMapping(uint8_t map)
{
    imuI2CWriteRegister(ADXL345_I2C_ADDR, ADXL345_INT_MAP, map);
}
