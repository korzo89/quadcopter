/*
 * bmp085.c
 *
 *  Created on: 25-09-2013
 *      Author: korzo
 */

//-----------------------------------------------------------------

#include "bmp085.h"
#include "i2c.h"

//-----------------------------------------------------------------

static int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
static uint16_t ac4, ac5, ac6;

static uint8_t buffer[2];

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
}

//-----------------------------------------------------------------

int16_t BMP085_readUT()
{

}

//-----------------------------------------------------------------

int32_t BMP085_readUP()
{

}

//-----------------------------------------------------------------

int16_t BMP085_readInt(uint8_t reg)
{
	I2CReadRegisterBurst(BMP085_I2C_ADDR, reg, buffer, 2);
	return ((int16_t) buffer[0] << 8) | buffer[1];
}

