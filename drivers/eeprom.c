/*
 * eeprom.c
 *
 *  Created on: 06-05-2014
 *      Author: Korzo
 */

#include "eeprom.h"

//-----------------------------------------------------------------

#include <inc/hw_types.h>
#include <driverlib/i2c.h>

#include <drivers/common_i2c.h>
#include <drivers/i2c.h>

//-----------------------------------------------------------------

#define EEPROM_I2C_ADDR         0x50

//-----------------------------------------------------------------

static bool eepromWriteStart(uint16_t addr)
{
    I2CMasterSlaveAddrSet(COMMON_I2C_BASE, EEPROM_I2C_ADDR, false);
    if (I2CDataPut(COMMON_I2C_BASE, addr >> 8, I2C_MASTER_CMD_BURST_SEND_START) != I2C_MASTER_ERR_NONE)
        return false;
    if (I2CDataPut(COMMON_I2C_BASE, addr & 0xFF, I2C_MASTER_CMD_BURST_SEND_CONT) != I2C_MASTER_ERR_NONE)
        return false;

    return true;
}

static bool eepromReadStart(unsigned long control)
{
    I2CMasterSlaveAddrSet(COMMON_I2C_BASE, EEPROM_I2C_ADDR, true);
    I2CMasterControl(COMMON_I2C_BASE, control);
    while (I2CMasterBusy(COMMON_I2C_BASE));
    if (I2CMasterErr(COMMON_I2C_BASE) != I2C_MASTER_ERR_NONE)
        return false;

    return true;
}

//-----------------------------------------------------------------

bool eepromWrite(uint16_t addr, uint8_t *buf, unsigned int len)
{
    if (!comI2CLock())
        return false;

    if (!eepromWriteStart(addr))
        return false;

    int i;
    for (i = 0; i < len - 1; i++)
    {
        I2CMasterDataPut(COMMON_I2C_BASE, buf[i]);
        I2CMasterControl(COMMON_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while (I2CMasterBusy(COMMON_I2C_BASE));
        if (I2CMasterErr(COMMON_I2C_BASE) != I2C_MASTER_ERR_NONE)
            return false;
    }

    I2CMasterDataPut(COMMON_I2C_BASE, buf[i]);
    I2CMasterControl(COMMON_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(COMMON_I2C_BASE));

    bool res = (I2CMasterErr(COMMON_I2C_BASE) == I2C_MASTER_ERR_NONE);

    comI2CUnlock();

    return res;
}

//-----------------------------------------------------------------

bool eepromWriteByte(uint16_t addr, uint8_t data)
{
    if (!comI2CLock())
        return false;

    if (!eepromWriteStart(addr))
        return false;

    I2CMasterDataPut(COMMON_I2C_BASE, data);
    I2CMasterControl(COMMON_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(COMMON_I2C_BASE));

    bool res = (I2CMasterErr(COMMON_I2C_BASE) == I2C_MASTER_ERR_NONE);

    comI2CUnlock();

    return res;
}

//-----------------------------------------------------------------

bool eepromRead(uint16_t addr, uint8_t *buf, unsigned int len)
{
    if (!comI2CLock())
        return false;

    if (!eepromWriteStart(addr))
        return false;

    if (!eepromReadStart(I2C_MASTER_CMD_BURST_RECEIVE_START))
        return false;

    buf[0] = I2CMasterDataGet(COMMON_I2C_BASE);

    int i;
    for (i = 1; i < len - 1; i++)
    {
        I2CMasterControl(COMMON_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while (I2CMasterBusy(COMMON_I2C_BASE));
        if (I2CMasterErr(COMMON_I2C_BASE) != I2C_MASTER_ERR_NONE)
            return false;
        buf[i] = I2CMasterDataGet(COMMON_I2C_BASE);
    }

    I2CMasterControl(COMMON_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(COMMON_I2C_BASE));
    if (I2CMasterErr(COMMON_I2C_BASE) != I2C_MASTER_ERR_NONE)
        return false;
    buf[i] = I2CMasterDataGet(COMMON_I2C_BASE);

    bool res = (I2CMasterErr(COMMON_I2C_BASE) == I2C_MASTER_ERR_NONE);

    comI2CUnlock();

    return res;
}

//-----------------------------------------------------------------

uint8_t eepromReadByte(uint16_t addr, bool *res)
{
    if (!comI2CLock())
        return false;

    if (!eepromWriteStart(addr))
        return false;

    if (!eepromReadStart(I2C_MASTER_CMD_SINGLE_RECEIVE))
        return false;

    while (I2CMasterBusy(COMMON_I2C_BASE));
    if (res)
        *res = (I2CMasterErr(COMMON_I2C_BASE) == I2C_MASTER_ERR_NONE);

    uint8_t data = I2CMasterDataGet(COMMON_I2C_BASE);

    comI2CUnlock();

    return data;
}
