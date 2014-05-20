/*
 * l3g4200d.c
 *
 *  Created on: 24-09-2013
 *      Author: Korzo
 */

#include "l3g4200d.h"
#include <drivers/imu_i2c.h>

//-----------------------------------------------------------------

#define L3G4200D_ADDR       0x69

//-----------------------------------------------------------------

static i2c_t *i2c_if;

static uint8_t buffer[6];

//-----------------------------------------------------------------

void l3g4200d_init(void)
{
    i2c_if = imu_i2c_get_if();

    l3g4200d_write_register(L3G4200D_CTRL_REG4, 0xA0);
    l3g4200d_write_register(L3G4200D_CTRL_REG1, 0x4F);
}

//-----------------------------------------------------------------

void l3g4200d_read_gyro(int16_t *x, int16_t *y, int16_t *z)
{
    l3g4200d_read_register(L3G4200D_OUT_X_L, buffer, 6);

    *x = ((int16_t) buffer[1] << 8) | buffer[0];
    *y = ((int16_t) buffer[3] << 8) | buffer[2];
    *z = ((int16_t) buffer[5] << 8) | buffer[4];
}

//-----------------------------------------------------------------

void l3g4200d_write_register(uint8_t reg, uint8_t data)
{
    i2c_write_reg_byte(i2c_if, L3G4200D_ADDR, reg, data);
}

//-----------------------------------------------------------------

uint8_t l3g4200d_read_register_byte(uint8_t reg)
{
    return i2c_read_reg_byte(i2c_if, L3G4200D_ADDR, reg, NULL);
}

//-----------------------------------------------------------------

void l3g4200d_read_register(uint8_t reg, uint8_t *buf, int len)
{
    i2c_read_reg(i2c_if, L3G4200D_ADDR, reg | 0x80, buf, len);
}
