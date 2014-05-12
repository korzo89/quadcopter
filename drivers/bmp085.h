/*
 * bmp085.h
 *
 *  Created on: 25-09-2013
 *      Author: korzo
 */

#ifndef BMP085_H_
#define BMP085_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

#define BMP085_I2C_ADDR	        0x77

#define BMP085_AC1_MSB          0xAA
#define BMP085_AC1_LSB          0xAB
#define BMP085_AC2_MSB          0xAC
#define BMP085_AC2_LSB          0xAD
#define BMP085_AC3_MSB          0xAE
#define BMP085_AC3_LSB          0xAF
#define BMP085_AC4_MSB          0xB0
#define BMP085_AC4_LSB          0xB1
#define BMP085_AC5_MSB          0xB2
#define BMP085_AC5_LSB          0xB3
#define BMP085_AC6_MSB          0xB4
#define BMP085_AC6_LSB          0xB5
#define BMP085_B1_MSB           0xB6
#define BMP085_B1_LSB           0xB7
#define BMP085_B2_MSB           0xB8
#define BMP085_B2_LSB           0xB9
#define BMP085_MB_MSB           0xBA
#define BMP085_MB_LSB           0xBB
#define BMP085_MC_MSB           0xBC
#define BMP085_MC_LSB           0xBD
#define BMP085_MD_MSB           0xBE
#define BMP085_MD_LSB           0xBF

#define BMP085_CONTROL          0xF4
#define BMP085_UT               0x2E
#define BMP085_UP               0x34

#define BMP085_DATA_MSB         0xF6
#define BMP085_DATA_LSB         0xF7
#define BMP085_DATA_XLSB        0xF8

#define BMP085_OSS              0

//-----------------------------------------------------------------

void bmp085Init();

int16_t bmp085ReadTemperature();
int32_t bmp085ReadPressure();

int16_t bmp085ReadUT();
int32_t bmp085ReadUP();

int16_t bmp085ReadInt(uint8_t reg);

//-----------------------------------------------------------------

#endif /* BMP085_H_ */
