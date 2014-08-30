/*
 * bmp085.c
 *
 *  Created on: 25-09-2013
 *      Author: korzo
 */

//-----------------------------------------------------------------

#include "bmp085.h"
#include <drivers/imu_i2c.h>
#include <utils/delay.h>

//-----------------------------------------------------------------

#define BMP085_ADDR         0x77

//-----------------------------------------------------------------

static i2c_t *i2c_if;

static int16_t ac1, ac2, ac3, b1, b2, mc, md; // , mb
static uint16_t ac4, ac5, ac6;
static int32_t b5;

static uint8_t buffer[3];

//-----------------------------------------------------------------

void bmp085_init(void)
{
    i2c_if = imu_i2c_get_if();

    ac1 = bmp085_read_int(BMP085_AC1_MSB);
    ac2 = bmp085_read_int(BMP085_AC2_MSB);
    ac3 = bmp085_read_int(BMP085_AC3_MSB);
    ac4 = bmp085_read_int(BMP085_AC4_MSB);
    ac5 = bmp085_read_int(BMP085_AC5_MSB);
    ac6 = bmp085_read_int(BMP085_AC6_MSB);
    b1 = bmp085_read_int(BMP085_B1_MSB);
    b2 = bmp085_read_int(BMP085_B2_MSB);
//    mb = bmp085ReadInt(BMP085_MB_MSB);
    mc = bmp085_read_int(BMP085_MC_MSB);
    md = bmp085_read_int(BMP085_MD_MSB);
//    ac1 = 408;
//    ac2 = -72;
//    ac3 = -14383;
//    ac4 = 32741;
//    ac5 = 32757;
//    ac6 = 23153;
//    b1 = 6190;
//    b2 = 4;
//    mb = -32768;
//    mc = -8711;
//    md = 2868;
}

//-----------------------------------------------------------------

int16_t bmp085_read_temp(void)
{
    int32_t x1, x2;
    int16_t ut;

    ut = bmp085_read_ut();
//    ut = 27898;

    x1 = (((int32_t) ut - ac6) * ac5) >> 15;
    x2 = ((int32_t) mc << 11) / (x1 + md);
    b5 = x1 + x2;

    return (b5 + 8) >> 4;
}

//-----------------------------------------------------------------

int32_t bmp085_read_pressure(void)
{
    int32_t x1, x2, x3, b3, b6, p, up;
    uint32_t b4, b7;

    up = bmp085_read_up();
//    up = 23843;

    b6 = b5 - 4000;
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = ((((int32_t) ac1 * 4 + x3) << BMP085_OSS) + 2) >> 2;

    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;

    b7 = ((uint32_t)up - b3) * (50000 >> BMP085_OSS);
    if (b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    return p;
}

//-----------------------------------------------------------------

int16_t bmp085_read_ut(void)
{
    i2c_write_reg_byte(i2c_if, BMP085_ADDR, BMP085_CONTROL, BMP085_UT);
    DELAY_MS(6);

    return bmp085_read_int(BMP085_DATA_MSB);
}

//-----------------------------------------------------------------

int32_t bmp085_read_up(void)
{
    i2c_write_reg_byte(i2c_if, BMP085_ADDR, BMP085_CONTROL, BMP085_UP + (BMP085_OSS << 6));
    DELAY_MS(3 + (3 << BMP085_OSS));

    i2c_read_reg(i2c_if, BMP085_ADDR, BMP085_DATA_MSB, buffer, 3);
    return (((int32_t)buffer[0] << 16) | ((int32_t)buffer[1] << 8) | buffer[2]) >> (8 - BMP085_OSS);
}

//-----------------------------------------------------------------

int16_t bmp085_read_int(uint8_t reg)
{
    i2c_read_reg(i2c_if, BMP085_ADDR, reg, buffer, 2);
    return ((int16_t)buffer[0] << 8) | buffer[1];
}

