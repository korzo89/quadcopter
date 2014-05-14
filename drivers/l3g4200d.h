/*
 * l3g4200d.h
 *
 *  Created on: 24-09-2013
 *      Author: Korzo
 */

#ifndef L3G4200D_H_
#define L3G4200D_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

#define L3G4200D_I2C_ADDR       0x69

#define L3G4200D_WHO_AM_I       0x0F
#define L3G4200D_CTRL_REG1      0x20
#define L3G4200D_CTRL_REG2      0x21
#define L3G4200D_CTRL_REG3      0x22
#define L3G4200D_CTRL_REG4      0x23
#define L3G4200D_CTRL_REG5      0x24
#define L3G4200D_REFERENCE      0x25
#define L3G4200D_OUT_TEMP       0x26
#define L3G4200D_STATUS_REG     0x27
#define L3G4200D_OUT_X_L        0x28
#define L3G4200D_OUT_X_H        0x29
#define L3G4200D_OUT_Y_L        0x2A
#define L3G4200D_OUT_Y_H        0x2B
#define L3G4200D_OUT_Z_L        0x2C
#define L3G4200D_OUT_Z_H        0x2D
#define L3G4200D_FIFO_CTRL_REG  0x2E
#define L3G4200D_FIFO_SRC_REG   0x2F
#define L3G4200D_INT1_CFG       0x30
#define L3G4200D_INT1_SRC       0x31
#define L3G4200D_INT1_TSH_XH    0x32
#define L3G4200D_INT1_TSH_XL    0x33
#define L3G4200D_INT1_TSH_YH    0x34
#define L3G4200D_INT1_TSH_YL    0x35
#define L3G4200D_INT1_TSH_ZH    0x36
#define L3G4200D_INT1_TSH_ZL    0x37
#define L3G4200D_INT1_DURATION  0x38

//-----------------------------------------------------------------

void l3g4200dInit();

void l3g4200dReadGyro(int16_t *x, int16_t *y, int16_t *z);

void l3g4200dWriteRegister(uint8_t reg, uint8_t data);

uint8_t l3g4200dReadRegister(uint8_t reg);
void l3g4200dReadRegisterBurst(uint8_t reg, uint8_t *buf, int len);

//-----------------------------------------------------------------

#endif /* L3G4200D_H_ */