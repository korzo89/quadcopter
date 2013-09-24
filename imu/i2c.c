/*
 * i2c.c
 *
 *  Created on: 22-09-2013
 *      Author: Korzo
 */

#include "i2c.h"

#include <inc/hw_types.h>
#include <driverlib/i2c.h>

//-----------------------------------------------------------------

void I2CWriteRegister(uint8_t addr, uint8_t reg, uint8_t data)
{
    I2CMasterSlaveAddrSet(I2C_BASE, addr, false);
    I2CMasterDataPut(I2C_BASE, reg);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C_BASE));

    I2CMasterDataPut(I2C_BASE, data);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C_BASE));
}

//-----------------------------------------------------------------

void I2CWriteRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    int i;

    I2CMasterSlaveAddrSet(I2C_BASE, addr, false);
    I2CMasterDataPut(I2C_BASE, reg);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C_BASE));

    for (i = 0; i < len - 1; i++)
    {
        I2CMasterDataPut(I2C_BASE, buf[i]);
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while (I2CMasterBusy(I2C_BASE));
    }

    I2CMasterDataPut(I2C_BASE, buf[i]);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C_BASE));
}

//-----------------------------------------------------------------

uint8_t I2CReadRegister(uint8_t addr, uint8_t reg)
{
    I2CMasterSlaveAddrSet(I2C_BASE, addr, false);
    I2CMasterDataPut(I2C_BASE, reg);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C_BASE));

    I2CMasterSlaveAddrSet(I2C_BASE, addr, true);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C_BASE));

    return I2CMasterDataGet(I2C_BASE);
}

//-----------------------------------------------------------------

void I2CReadRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    int i;

    I2CMasterSlaveAddrSet(I2C_BASE, addr, false);
    I2CMasterDataPut(I2C_BASE, reg);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C_BASE));

    I2CMasterSlaveAddrSet(I2C_BASE, addr, true);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (I2CMasterBusy(I2C_BASE));
    buf[0] = I2CMasterDataGet(I2C_BASE);

    for (i = 1; i < len - 1; i++)
    {
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while (I2CMasterBusy(I2C_BASE));
        buf[i] = I2CMasterDataGet(I2C_BASE);
    }

    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(I2C_BASE));
    buf[i] = I2CMasterDataGet(I2C_BASE);
}


