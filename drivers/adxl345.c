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

#define ADXL345_ADDR        0x53

//-----------------------------------------------------------------

static i2c_t *i2c_if;

static uint8_t buffer[6];

//-----------------------------------------------------------------

void adxl345_init(void)
{
    i2c_if = imu_i2c_get_if();

    i2c_write_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_POWER_CTL, 0x00);
    i2c_write_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_POWER_CTL, ADXL345_MEASURE);
}

//-----------------------------------------------------------------

void adxl345_get_accel(int16_t *x, int16_t *y, int16_t *z)
{
    i2c_read_reg(i2c_if, ADXL345_ADDR, ADXL345_DATAX0, buffer, 6);

    *x = ((int16_t)buffer[1] << 8) | buffer[0];
    *y = ((int16_t)buffer[3] << 8) | buffer[2];
    *z = ((int16_t)buffer[5] << 8) | buffer[4];
}

//-----------------------------------------------------------------

enum adxl345_range adxl345_get_range()
{
    uint8_t range = i2c_read_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_DATA_FORMAT, NULL);
    return (enum adxl345_range)(range & ADXL345_RANGE_MASK);
}

//-----------------------------------------------------------------

void adxl345_set_range(enum adxl345_range range)
{
    uint8_t reg = i2c_read_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_POWER_CTL, NULL);
    reg = (reg & ~ADXL345_RANGE_MASK) | (uint8_t)range;

    i2c_write_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_POWER_CTL, reg);
}

//-----------------------------------------------------------------

void adxl345_get_offsets(int8_t *x, int8_t *y, int8_t *z)
{
    i2c_read_reg(i2c_if, ADXL345_ADDR, ADXL345_OFSX, buffer, 3);

    *x = (int8_t)buffer[0];
    *y = (int8_t)buffer[1];
    *z = (int8_t)buffer[2];
}

//-----------------------------------------------------------------

void adxl345_set_offsets(int8_t x, int8_t y, int8_t z)
{
    i2c_write_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_OFSX, x);
    i2c_write_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_OFSY, y);
    i2c_write_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_OFSZ, z);
}

//-----------------------------------------------------------------

uint8_t adxl345_get_int_source()
{
    return i2c_read_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_INT_SOURCE, NULL);
}

//-----------------------------------------------------------------

uint8_t adxl345_get_int_mapping()
{
    return i2c_read_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_INT_MAP, NULL);
}

//-----------------------------------------------------------------

void adxl345_set_int_mapping(uint8_t map)
{
    i2c_write_reg_byte(i2c_if, ADXL345_ADDR, ADXL345_INT_MAP, map);
}
