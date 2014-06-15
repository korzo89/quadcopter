/*
 * i2c.h
 *
 *  Created on: 22-09-2013
 *      Author: Korzo
 */

#ifndef I2C_H_
#define I2C_H_

//-----------------------------------------------------------------

#include <defs.h>

#include <FreeRTOS.h>
#include <semphr.h>

//-----------------------------------------------------------------

struct i2c_pin_cfg
{
    uint32_t port;
    uint32_t pin;
    uint32_t sysctl;
    uint32_t conf;
};

struct i2c_cfg
{
    bool fast;
    uint32_t base;
    uint32_t sysctl;
    struct i2c_pin_cfg scl;
    struct i2c_pin_cfg sda;
};

struct i2c
{
    xSemaphoreHandle mutex;
    struct i2c_cfg conf;
};

typedef struct i2c i2c_t;

//-----------------------------------------------------------------

result_t i2c_init(i2c_t *obj);

result_t i2c_excl_take(i2c_t *obj);
result_t i2c_excl_give(i2c_t *obj);

result_t i2c_transfer(i2c_t *obj, uint8_t addr,
        uint8_t *wr, uint32_t wr_len, uint8_t *rd, uint32_t rd_len);

result_t i2c_read_reg(i2c_t *obj, uint8_t addr, uint8_t reg, uint8_t *rd, uint32_t rd_len);
result_t i2c_write_reg(i2c_t *obj, uint8_t addr, uint8_t reg, uint8_t *wr, uint32_t wr_len);

uint8_t i2c_read_reg_byte(i2c_t *obj, uint8_t addr, uint8_t reg, result_t *res);
result_t i2c_write_reg_byte(i2c_t *obj, uint8_t addr, uint8_t reg, uint8_t data);

result_t i2c_read_reg16(i2c_t *obj, uint8_t addr, uint16_t reg, uint8_t *rd, uint32_t rd_len);
result_t i2c_write_reg16(i2c_t *obj, uint8_t addr, uint16_t reg, uint8_t *wr, uint32_t wr_len);

result_t i2c_poll_ack(i2c_t *obj, uint8_t addr);

//-----------------------------------------------------------------

#endif /* I2C_H_ */
