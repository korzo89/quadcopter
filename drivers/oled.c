#include "oled.h"
#include "oled_font.h"
#include <drivers/common_i2c.h>
#include <utils/delay.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/pin_map.h>
#include <driverlib/gpio.h>
#include <driverlib/i2c.h>

//-----------------------------------------------------------------

#define OLED_ADDRESS    0x3C

//-----------------------------------------------------------------

static uint8_t currRow = 0, currCol = 0;

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
    currRow = row % OLED_ROWS;
    currCol = col % OLED_COLS;

    oledSendCmd(0xB0 + currRow);
    oledSendCmd(0x00 + (8 * currCol & 0x0F));
    oledSendCmd(0x10 + ((8 * currCol >> 4) & 0x0F));
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
    if (c == '\n')
    {
        oledSetPos(currRow + 1, 0);
        return;
    }

    int i;
    for (i = 0; i < 8; ++i)
        oledSendData(OLED_FONT[c - 0x20][i]);
}

//-----------------------------------------------------------------

bool oledSendData(uint8_t data)
{
    if (comI2CWriteRegister(OLED_ADDRESS, 0x40, data) == I2C_MASTER_ERR_NONE)
    {
        currCol++;
        if (currCol == OLED_COLS)
            oledSetPos(currRow + 1, 0);
        return true;
    }
    return false;
}

//-----------------------------------------------------------------

bool oledSendCmd(uint8_t cmd)
{
    return comI2CWriteRegister(OLED_ADDRESS, 0x80, cmd) == I2C_MASTER_ERR_NONE;
}

