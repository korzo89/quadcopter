#include "i2c.h"

#include <stellaris_config.h>
#include <portmacro.h>

//-----------------------------------------------------------------

result_t i2c_init(i2c_t *obj) {
    if (!obj)
        return RES_ERR_BAD_PARAM;

    obj->mutex = xSemaphoreCreateMutex();

    SysCtlPeripheralEnable(obj->conf.sysctl);
    SysCtlPeripheralEnable(obj->conf.scl.sysctl);
    SysCtlPeripheralEnable(obj->conf.sda.sysctl);

    GPIOPinConfigure(obj->conf.scl.conf);
    GPIOPinConfigure(obj->conf.sda.conf);
    GPIOPinTypeI2CSCL(obj->conf.scl.port, obj->conf.scl.pin);
    GPIOPinTypeI2C(obj->conf.sda.port, obj->conf.sda.pin);

    I2CMasterInitExpClk(obj->conf.base, SysCtlClockGet(), obj->conf.fast);

    return RES_OK;
}

//-----------------------------------------------------------------

result_t i2c_excl_take(i2c_t *obj) {
    if (!obj)
        return RES_ERR_BAD_PARAM;

    if (xSemaphoreTakeRecursive(obj->mutex, portMAX_DELAY) != pdTRUE)
        return RES_ERR_IO;

    return RES_OK;
}

//-----------------------------------------------------------------

result_t i2c_excl_give(i2c_t *obj) {
    if (!obj)
        return RES_ERR_BAD_PARAM;

    xSemaphoreGiveRecursive(obj->mutex);
    return RES_OK;
}

//-----------------------------------------------------------------

static result_t i2c_write_byte(i2c_t *obj, uint8_t data, uint32_t ctrl) {
    uint32_t base = obj->conf.base;
    I2CMasterDataPut(base, data);
    I2CMasterControl(base, ctrl);
    while (I2CMasterBusy(base))
        ;
    if (I2CMasterErr(base) != I2C_MASTER_ERR_NONE)
        return RES_ERR_IO;

    return RES_OK;
}

//-----------------------------------------------------------------

static result_t i2c_write(i2c_t *obj, uint8_t addr, uint8_t *wr, uint32_t wr_len)
{
    uint32_t base = obj->conf.base;
    I2CMasterSlaveAddrSet(base, addr, false);

    uint32_t i = wr_len;
    while (i--)
    {
        uint32_t ctrl;
        if (wr_len == 1)
            ctrl = I2C_MASTER_CMD_SINGLE_SEND;
        else if (i == wr_len - 1)
            ctrl = I2C_MASTER_CMD_BURST_SEND_START;
        else if (i == 0)
            ctrl = I2C_MASTER_CMD_BURST_SEND_FINISH;
        else
            ctrl = I2C_MASTER_CMD_BURST_SEND_CONT;

        if (i2c_write_byte(obj, *wr++, ctrl) != RES_OK)
            return RES_ERR_IO;
    }

    return RES_OK;
}

//-----------------------------------------------------------------

static result_t i2c_read(i2c_t *obj, uint8_t addr, uint8_t *rd, uint32_t rd_len)
{
    uint32_t base = obj->conf.base;
    I2CMasterSlaveAddrSet(base, addr, true);

    uint32_t i = rd_len;
    while (i--)
    {
        uint32_t ctrl;
        if (rd_len == 1)
            ctrl = I2C_MASTER_CMD_SINGLE_RECEIVE;
        else if (i == rd_len - 1)
            ctrl = I2C_MASTER_CMD_BURST_RECEIVE_START;
        else if (i == 0)
            ctrl = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
        else
            ctrl = I2C_MASTER_CMD_BURST_RECEIVE_CONT;

        I2CMasterControl(base, ctrl);
        while (I2CMasterBusy(base))
            ;
        if (I2CMasterErr(base) != I2C_MASTER_ERR_NONE)
            return RES_ERR_IO;

        *rd++ = I2CMasterDataGet(base);
    }

    return RES_OK;
}

//-----------------------------------------------------------------

static result_t i2c_transfer_internal(i2c_t *obj, uint8_t addr,
        uint8_t *wr, uint32_t wr_len, uint8_t *rd, uint32_t rd_len)
{
    result_t res;

    if (wr && wr_len > 0)
    {
        res = i2c_write(obj, addr, wr, wr_len);
        if (res != RES_OK)
            return res;
    }

    if (rd && rd_len > 0)
    {
        res = i2c_read(obj, addr, rd, rd_len);
        if (res != RES_OK)
            return res;
    }

    return RES_OK;
}

//-----------------------------------------------------------------

result_t i2c_transfer(i2c_t *obj, uint8_t addr,
        uint8_t *wr, uint32_t wr_len, uint8_t *rd, uint32_t rd_len)
{
    if (!obj)
        return RES_ERR_BAD_PARAM;

    result_t res = i2c_excl_take(obj);
    if (res != RES_OK)
        return res;

    res = i2c_transfer_internal(obj, addr, wr, wr_len, rd, rd_len);
    if (res != RES_OK)
        return res;

    i2c_excl_give(obj);

    return RES_OK;
}

//-----------------------------------------------------------------

result_t i2c_read_reg(i2c_t *obj, uint8_t addr, uint8_t reg, uint8_t *rd, uint32_t rd_len)
{
    if (!obj || !rd || rd_len == 0)
        return RES_ERR_BAD_PARAM;

    return i2c_transfer(obj, addr, &reg, 1, rd, rd_len);
}

//-----------------------------------------------------------------

result_t i2c_write_reg(i2c_t *obj, uint8_t addr, uint8_t reg, uint8_t *wr, uint32_t wr_len)
{
    if (!obj || !wr || wr_len == 0)
        return RES_ERR_BAD_PARAM;

    result_t res = i2c_excl_take(obj);
    if (res != RES_OK)
        return res;

    uint32_t base = obj->conf.base;
    I2CMasterSlaveAddrSet(base, addr, false);

    if (i2c_write_byte(obj, reg, I2C_MASTER_CMD_BURST_SEND_START) != RES_OK)
        return RES_ERR_IO;

    uint32_t i = wr_len;
    while (i--)
    {
        uint32_t ctrl;
        if (wr_len == 1 || i == 0)
            ctrl = I2C_MASTER_CMD_BURST_SEND_FINISH;
        else
            ctrl = I2C_MASTER_CMD_BURST_SEND_CONT;

        if (i2c_write_byte(obj, *wr++, ctrl) != RES_OK)
            return RES_ERR_IO;
    }

    i2c_excl_give(obj);

    return RES_OK;
}

//-----------------------------------------------------------------

result_t i2c_read_reg16(i2c_t *obj, uint8_t addr, uint16_t reg, uint8_t *rd, uint32_t rd_len)
{
    if (!obj || !rd || rd_len == 0)
        return RES_ERR_BAD_PARAM;

    uint8_t reg_wr[] = { reg >> 8, reg & 0x0F };
    return i2c_transfer(obj, addr, reg_wr, 2, rd, rd_len);
}

//-----------------------------------------------------------------

result_t i2c_write_reg16(i2c_t *obj, uint8_t addr, uint16_t reg, uint8_t *wr, uint32_t wr_len)
{
    if (!obj || !wr || wr_len == 0)
        return RES_ERR_BAD_PARAM;

    result_t res = i2c_excl_take(obj);
    if (res != RES_OK)
        return res;

    uint32_t base = obj->conf.base;
    I2CMasterSlaveAddrSet(base, addr, false);

    if (i2c_write_byte(obj, reg >> 8, I2C_MASTER_CMD_BURST_SEND_START) != RES_OK)
        return RES_ERR_IO;
    if (i2c_write_byte(obj, reg & 0x0F, I2C_MASTER_CMD_BURST_SEND_CONT) != RES_OK)
        return RES_ERR_IO;

    uint32_t i = wr_len;
    while (i--)
    {
        uint32_t ctrl;
        if (wr_len == 1 || i == 0)
            ctrl = I2C_MASTER_CMD_BURST_SEND_FINISH;
        else
            ctrl = I2C_MASTER_CMD_BURST_SEND_CONT;

        if (i2c_write_byte(obj, *wr++, ctrl) != RES_OK)
            return RES_ERR_IO;
    }

    i2c_excl_give(obj);

    return RES_OK;
}

//-----------------------------------------------------------------

unsigned long i2cWriteRegister(unsigned long base, uint8_t addr, uint8_t reg, uint8_t data)
{
    unsigned long err;

    I2CMasterSlaveAddrSet(base, addr, false);
    if ((err = i2cDataPut(base, reg, I2C_MASTER_CMD_BURST_SEND_START))
            != I2C_MASTER_ERR_NONE)
        return err;

    I2CMasterDataPut(base, data);
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(base))
        ;

    return I2CMasterErr(base);
}

//-----------------------------------------------------------------

unsigned long i2cWriteRegisterBurst(unsigned long base, uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    int i;
    unsigned long err;

    I2CMasterSlaveAddrSet(base, addr, false);
    if ((err = i2cDataPut(base, reg, I2C_MASTER_CMD_BURST_SEND_START))
            != I2C_MASTER_ERR_NONE)
        return err;

    for (i = 0; i < len - 1; i++)
    {
        I2CMasterDataPut(base, buf[i]);
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_CONT);
        while (I2CMasterBusy(base))
            ;
        if ((err = I2CMasterErr(base)) != I2C_MASTER_ERR_NONE)
            return err;
    }

    I2CMasterDataPut(base, buf[i]);
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(base))
        ;

    return I2CMasterErr(base);
}

//-----------------------------------------------------------------

uint8_t i2cReadRegister(unsigned long base, uint8_t addr, uint8_t reg,
        unsigned long *res)
{
    unsigned long err;

    I2CMasterSlaveAddrSet(base, addr, false);
    if ((err = i2cDataPut(base, reg, I2C_MASTER_CMD_SINGLE_SEND))
            != I2C_MASTER_ERR_NONE)
    {
        if (res)
            *res = err;
        return 0;
    }

    I2CMasterSlaveAddrSet(base, addr, true);
    I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(base))
        ;
    if (res)
        *res = I2CMasterErr(base);

    return I2CMasterDataGet(base);
}

//-----------------------------------------------------------------

unsigned long i2cReadRegisterBurst(unsigned long base, uint8_t addr,
        uint8_t reg, uint8_t *buf, int len)
{
    int i;
    unsigned long err;

    I2CMasterSlaveAddrSet(base, addr, false);
    if ((err = i2cDataPut(base, reg, I2C_MASTER_CMD_BURST_SEND_START))
            != I2C_MASTER_ERR_NONE)
        return err;

    I2CMasterSlaveAddrSet(base, addr, true);
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (I2CMasterBusy(base))
        ;
    if ((err = I2CMasterErr(base)) != I2C_MASTER_ERR_NONE)
        return err;

    buf[0] = I2CMasterDataGet(base);
    for (i = 1; i < len - 1; i++)
    {
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while (I2CMasterBusy(base))
            ;
        if ((err = I2CMasterErr(base)) != I2C_MASTER_ERR_NONE)
            return err;
        buf[i] = I2CMasterDataGet(base);
    }

    I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(base))
        ;
    if ((err = I2CMasterErr(base)) != I2C_MASTER_ERR_NONE)
        return err;
    buf[i] = I2CMasterDataGet(base);

    return I2CMasterErr(base);
}

//-----------------------------------------------------------------

unsigned long i2cDataPut(unsigned long base, uint8_t data,
        unsigned long control)
{
    I2CMasterDataPut(base, data);
    I2CMasterControl(base, control);
    while (I2CMasterBusy(base));

    return I2CMasterErr(base);
}

