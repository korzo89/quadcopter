/*
 * hmc5883.c
 *
 *  Created on: 24-09-2013
 *      Author: Korzo
 */

#include "hmc5883.h"
#include <drivers/imu_i2c.h>

//-----------------------------------------------------------------

#define HMC5883_ADDR        0x1E

//-----------------------------------------------------------------

static i2c_t *i2c_if;

static uint8_t buffer[6];

//-----------------------------------------------------------------

void hmc5883_init()
{
    i2c_if = imu_i2c_get_if();

    i2c_write_reg_byte(i2c_if, HMC5883_ADDR, HMC5883_MODE, 0x00);
}

//-----------------------------------------------------------------

void hmc5883_read_mag(int16_t *x, int16_t *y, int16_t *z)
{
    i2c_read_reg(i2c_if, HMC5883_ADDR, HMC5883_OUT_X_MSB, buffer, 6);

    *x = ((int16_t) buffer[0] << 8) | buffer[1];
    *z = ((int16_t) buffer[2] << 8) | buffer[3];
    *y = ((int16_t) buffer[4] << 8) | buffer[5];
}

