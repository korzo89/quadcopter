#include "common_i2c.h"

#include <FreeRTOS.h>
#include <semphr.h>
#include <portmacro.h>

#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/i2c.h>

//-----------------------------------------------------------------

static xSemaphoreHandle i2cSemaphore;

//-----------------------------------------------------------------

void comI2CInit(void)
{
    i2cSemaphore = xSemaphoreCreateMutex();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

    I2CMasterInitExpClk(COMMON_I2C_BASE, SysCtlClockGet(), true);
}

//-----------------------------------------------------------------

unsigned long comI2CWriteRegister(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) != pdTRUE)
        return COMMON_I2C_ERR_BLOCKED;

    unsigned long res = I2CWriteRegister(COMMON_I2C_BASE, addr, reg, data);
    xSemaphoreGive(i2cSemaphore);

    return res;
}

//-----------------------------------------------------------------

unsigned long comI2CWriteRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) != pdTRUE)
        return COMMON_I2C_ERR_BLOCKED;

    unsigned long res = I2CWriteRegisterBurst(COMMON_I2C_BASE, addr, reg, buf, len);
    xSemaphoreGive(i2cSemaphore);

    return res;
}

//-----------------------------------------------------------------

uint8_t comI2CReadRegister(uint8_t addr, uint8_t reg, unsigned long *res)
{
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) != pdTRUE)
    {
        if (res)
            *res = COMMON_I2C_ERR_BLOCKED;
        return 0;
    }

    uint8_t num = I2CReadRegister(COMMON_I2C_BASE, addr, reg, res);
    xSemaphoreGive(i2cSemaphore);

    return num;
}

//-----------------------------------------------------------------

unsigned long comI2CReadRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) != pdTRUE)
        return COMMON_I2C_ERR_BLOCKED;

    unsigned long res = I2CReadRegisterBurst(COMMON_I2C_BASE, addr, reg, buf, len);
    xSemaphoreGive(i2cSemaphore);

    return res;
}

//-----------------------------------------------------------------

bool comI2CLock(void)
{
    return xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE;
}

//-----------------------------------------------------------------

void comI2CUnlock(void)
{
    xSemaphoreGive(i2cSemaphore);
}
