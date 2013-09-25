/*
 * bmp085.c
 *
 *  Created on: 25-09-2013
 *      Author: korzo
 */

//-----------------------------------------------------------------

#include "bmp085.h"
#include "i2c.h"
#include "../utils/utils.h"

//-----------------------------------------------------------------

static int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
static uint16_t ac4, ac5, ac6;
static int32_t b5;

static uint8_t buffer[3];

//-----------------------------------------------------------------

void BMP085_init()
{
    ac1 = BMP085_readInt(BMP085_AC1_MSB);
    ac2 = BMP085_readInt(BMP085_AC2_MSB);
    ac3 = BMP085_readInt(BMP085_AC3_MSB);
    ac4 = BMP085_readInt(BMP085_AC4_MSB);
    ac5 = BMP085_readInt(BMP085_AC5_MSB);
    ac6 = BMP085_readInt(BMP085_AC6_MSB);
    b1 = BMP085_readInt(BMP085_B1_MSB);
    b2 = BMP085_readInt(BMP085_B2_MSB);
    mb = BMP085_readInt(BMP085_MB_MSB);
    mc = BMP085_readInt(BMP085_MC_MSB);
    md = BMP085_readInt(BMP085_MD_MSB);
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

int16_t BMP085_readTemperature()
{
    int32_t x1, x2;
    int16_t ut;

    ut = BMP085_readUT();
//    ut = 27898;

    x1 = (((int32_t) ut - ac6) * ac5) >> 15;
    x2 = ((int32_t) mc << 11) / (x1 + md);
    b5 = x1 + x2;

    return (b5 + 8) >> 4;
}

//-----------------------------------------------------------------

int32_t BMP085_readPressure()
{
    int32_t x1, x2, x3, b3, b6, p, up;
    uint32_t b4, b7;

    up = BMP085_readUP();
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

    b7 = ((uint32_t) up - b3) * (50000 >> BMP085_OSS);
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

int16_t BMP085_readUT()
{
    I2CWriteRegister(BMP085_I2C_ADDR, BMP085_CONTROL, BMP085_UT);
    delay(5500UL);

    return BMP085_readInt(BMP085_DATA_MSB);
}

//-----------------------------------------------------------------

int32_t BMP085_readUP()
{
    I2CWriteRegister(BMP085_I2C_ADDR, BMP085_CONTROL, BMP085_UP + (BMP085_OSS << 6));
    delay(2500UL + (3000UL << BMP085_OSS));

    I2CReadRegisterBurst(BMP085_I2C_ADDR, BMP085_DATA_MSB, buffer, 3);
    return (((int32_t) buffer[0] << 16) | ((int32_t) buffer[1] << 8) | buffer[2]) >> (8 - BMP085_OSS);
}

//-----------------------------------------------------------------

int16_t BMP085_readInt(uint8_t reg)
{
    I2CReadRegisterBurst(BMP085_I2C_ADDR, reg, buffer, 2);
    return ((int16_t) buffer[0] << 8) | buffer[1];
}

