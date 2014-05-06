/*
 * oled.c
 *
 *  Created on: 24-04-2014
 *      Author: Korzo
 */

#include "oled.h"
#include "oled_font.h"
#include <i2c.h>
#include <utils/utils.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/pin_map.h>
#include <driverlib/gpio.h>
#include <driverlib/i2c.h>

//-----------------------------------------------------------------

#define OLED_I2C_BASE   I2C2_MASTER_BASE
#define OLED_ADDRESS    0x3C

//-----------------------------------------------------------------

void oledConfig(void)
{
    // I2C config
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

    I2CMasterInitExpClk(OLED_I2C_BASE, SysCtlClockGet(), true);
}

//-----------------------------------------------------------------

bool oledInit(void)
{
    if (!oledSendCmd(0xAE))
        return false;
    oledSendCmd(0x2E);
    oledSendCmd(0xA4);
    delay(10);
    oledSendCmd(0xA1);
    oledSendCmd(0xC8);
    oledSendCmd(0xAF);
    oledSendCmd(0x20);
    oledSendCmd(0x02);
    oledSendCmd(0xA6);
    delay(10);
    oledClear();

    return true;
}

//-----------------------------------------------------------------

void oledClear(void)
{
    int i, j;
    for (i = 0; i < 8; ++i)
    {
        oledSetPos(i, 0);
        for (j = 0; j < 128; ++j)
            oledSendData(0x00);
    }
    oledSetPos(0, 0);
}

//-----------------------------------------------------------------

void oledSetPos(uint8_t row, uint8_t col)
{
    oledSendCmd(0xB0 + row);
    oledSendCmd(0x00 + (8 * col & 0x0F));
    oledSendCmd(0x10 + ((8 * col >> 4) & 0x0F));
}

//-----------------------------------------------------------------

void oledDispStr(char *str)
{
    char c;
    while (c = *str++)
        oledDispChar(c);
}

//-----------------------------------------------------------------

void oledDispStrAt(char *str, uint8_t row, uint8_t col)
{
    oledSetPos(row, col);
    oledDispStr(str);
}

//-----------------------------------------------------------------

void oledDispChar(char c)
{
    int i;
    for (i = 0; i < 8; ++i)
        oledSendData(OLED_FONT[c - 0x20][i]);
}

//-----------------------------------------------------------------

bool oledSendData(uint8_t data)
{
    return I2CWriteRegister(OLED_I2C_BASE, OLED_ADDRESS, 0x40, data) == I2C_MASTER_ERR_NONE;
}

//-----------------------------------------------------------------

bool oledSendCmd(uint8_t cmd)
{
    return I2CWriteRegister(OLED_I2C_BASE, OLED_ADDRESS, 0x80, cmd) == I2C_MASTER_ERR_NONE;
}

