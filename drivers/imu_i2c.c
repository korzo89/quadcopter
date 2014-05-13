#include "imu_i2c.h"

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

void imuI2CInit(void)
{
    i2cSemaphore = xSemaphoreCreateMutex();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(), false);
}

//-----------------------------------------------------------------

unsigned long imuI2CWriteRegister(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) != pdTRUE)
        return IMU_I2C_ERR_BLOCKED;

    unsigned long res = i2cWriteRegister(IMU_I2C_BASE, addr, reg, data);
    xSemaphoreGive(i2cSemaphore);

    return res;
}

//-----------------------------------------------------------------

unsigned long imuI2CWriteRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) != pdTRUE)
        return IMU_I2C_ERR_BLOCKED;

    unsigned long res = i2cWriteRegisterBurst(IMU_I2C_BASE, addr, reg, buf, len);
    xSemaphoreGive(i2cSemaphore);

    return res;
}

//-----------------------------------------------------------------

uint8_t imuI2CReadRegister(uint8_t addr, uint8_t reg, unsigned long *res)
{
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) != pdTRUE)
    {
        if (res)
            *res = IMU_I2C_ERR_BLOCKED;
        return 0;
    }

    uint8_t num = i2cReadRegister(IMU_I2C_BASE, addr, reg, res);
    xSemaphoreGive(i2cSemaphore);

    return num;
}

//-----------------------------------------------------------------

unsigned long imuI2CReadRegisterBurst(uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) != pdTRUE)
        return IMU_I2C_ERR_BLOCKED;

    unsigned long res = i2cReadRegisterBurst(IMU_I2C_BASE, addr, reg, buf, len);
    xSemaphoreGive(i2cSemaphore);

    return res;
}

//-----------------------------------------------------------------

bool imuI2CLock(void)
{
    return xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE;
}

//-----------------------------------------------------------------

void imuI2CUnlock(void)
{
    xSemaphoreGive(i2cSemaphore);
}
