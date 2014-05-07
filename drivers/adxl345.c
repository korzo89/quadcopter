/*
 * adxl345.c
 *
 *  Created on: 23-09-2013
 *      Author: Korzo
 */

//-----------------------------------------------------------------

#include "adxl345.h"
#include <drivers/i2c.h>
#include <stdlib.h>

//-----------------------------------------------------------------

#define ADXL345_I2C_BASE        I2C0_MASTER_BASE

//-----------------------------------------------------------------

static uint8_t buffer[6];

//-----------------------------------------------------------------

void ADXL345_init()
{
    I2CWriteRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_POWER_CTL, 0x00);
    I2CWriteRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_POWER_CTL, ADXL345_MEASURE);
}

//-----------------------------------------------------------------

void ADXL345_getAcceleration(int16_t *x, int16_t *y, int16_t *z)
{
    I2CReadRegisterBurst(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_DATAX0, buffer, 6);

    *x = ((int16_t)buffer[1] << 8) | buffer[0];
    *y = ((int16_t)buffer[3] << 8) | buffer[2];
    *z = ((int16_t)buffer[5] << 8) | buffer[4];
}

//-----------------------------------------------------------------

ADXL345Range ADXL345_getRange()
{
    return (ADXL345Range)(I2CReadRegister(ADXL345_I2C_BASE,
            ADXL345_I2C_ADDR, ADXL345_DATA_FORMAT, NULL) & ADXL345_RANGE_MASK);
}

//-----------------------------------------------------------------

void ADXL345_setRange(ADXL345Range range)
{
    uint8_t reg = I2CReadRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_POWER_CTL, NULL);
    reg = (reg & ~ADXL345_RANGE_MASK) | (uint8_t)range;

    I2CWriteRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_POWER_CTL, reg);
}

//-----------------------------------------------------------------

void ADXL345_getOffsets(int8_t *x, int8_t *y, int8_t *z)
{
    I2CReadRegisterBurst(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_OFSX, buffer, 3);

    *x = (int8_t)buffer[0];
    *y = (int8_t)buffer[1];
    *z = (int8_t)buffer[2];
}

//-----------------------------------------------------------------

void ADXL345_setOffsets(int8_t x, int8_t y, int8_t z)
{
    I2CWriteRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_OFSX, x);
    I2CWriteRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_OFSY, y);
    I2CWriteRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_OFSZ, z);
}

//-----------------------------------------------------------------

uint8_t ADXL345_getIntSource()
{
    return I2CReadRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_INT_SOURCE, NULL);
}

//-----------------------------------------------------------------

uint8_t ADXL345_getIntMapping()
{
    return I2CReadRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_INT_MAP, NULL);
}

//-----------------------------------------------------------------

void ADXL345_setIntMapping(uint8_t map)
{
    I2CWriteRegister(ADXL345_I2C_BASE, ADXL345_I2C_ADDR, ADXL345_INT_MAP, map);
}
