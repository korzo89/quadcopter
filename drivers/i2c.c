#include "i2c.h"

#include <inc/hw_types.h>
#include <driverlib/i2c.h>
#include <stdbool.h>

//-----------------------------------------------------------------

unsigned long I2CWriteRegister(unsigned long base, uint8_t addr, uint8_t reg, uint8_t data)
{
    unsigned long err;

    I2CMasterSlaveAddrSet(base, addr, false);
    if ((err = I2CDataPut(base, reg, I2C_MASTER_CMD_BURST_SEND_START)) != I2C_MASTER_ERR_NONE)
        return err;

    I2CMasterDataPut(base, data);
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(base));

    return I2CMasterErr(base);
}

//-----------------------------------------------------------------

unsigned long I2CWriteRegisterBurst(unsigned long base, uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    int i;
    unsigned long err;

    I2CMasterSlaveAddrSet(base, addr, false);
    if ((err = I2CDataPut(base, reg, I2C_MASTER_CMD_BURST_SEND_START)) != I2C_MASTER_ERR_NONE)
        return err;

    for (i = 0; i < len - 1; i++)
    {
        I2CMasterDataPut(base, buf[i]);
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_CONT);
        while (I2CMasterBusy(base));
        if ((err = I2CMasterErr(base)) != I2C_MASTER_ERR_NONE)
            return err;
    }

    I2CMasterDataPut(base, buf[i]);
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(base));

    return I2CMasterErr(base);
}

//-----------------------------------------------------------------

uint8_t I2CReadRegister(unsigned long base, uint8_t addr, uint8_t reg, unsigned long *res)
{
    unsigned long err;

    I2CMasterSlaveAddrSet(base, addr, false);
    if ((err = I2CDataPut(base, reg, I2C_MASTER_CMD_SINGLE_SEND)) != I2C_MASTER_ERR_NONE)
    {
        if (res)
            *res = err;
        return 0;
    }

    I2CMasterSlaveAddrSet(base, addr, true);
    I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(base));
    if (res)
        *res = I2CMasterErr(base);

    return I2CMasterDataGet(base);
}

//-----------------------------------------------------------------

unsigned long I2CReadRegisterBurst(unsigned long base, uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    int i;
    unsigned long err;

    I2CMasterSlaveAddrSet(base, addr, false);
    if ((err = I2CDataPut(base, reg, I2C_MASTER_CMD_BURST_SEND_START)) != I2C_MASTER_ERR_NONE)
        return err;

    I2CMasterSlaveAddrSet(base, addr, true);
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (I2CMasterBusy(base));
    if ((err = I2CMasterErr(base)) != I2C_MASTER_ERR_NONE)
        return err;

    buf[0] = I2CMasterDataGet(base);
    for (i = 1; i < len - 1; i++)
    {
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while (I2CMasterBusy(base));
        if ((err = I2CMasterErr(base)) != I2C_MASTER_ERR_NONE)
            return err;
        buf[i] = I2CMasterDataGet(base);
    }

    I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(base));
    if ((err = I2CMasterErr(base)) != I2C_MASTER_ERR_NONE)
        return err;
    buf[i] = I2CMasterDataGet(base);

    return I2CMasterErr(base);
}

//-----------------------------------------------------------------

unsigned long I2CDataPut(unsigned long base, uint8_t data, unsigned long control)
{
    I2CMasterDataPut(base, data);
    I2CMasterControl(base, control);
    while (I2CMasterBusy(base));

    return I2CMasterErr(base);
}


