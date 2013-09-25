/*
 * nrf24_port.c
 *
 *  Created on: 18-08-2013
 *      Author: Korzo
 */

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"

#include "nrf24.h"
#include "../utils/utils.h"

//-----------------------------------------------------------------

#define NRF24_CE_PORT_BASE      GPIO_PORTA_BASE
#define NRF24_CE_PIN            GPIO_PIN_3

#define NRF24_CSN_PORT_BASE     GPIO_PORTA_BASE
#define NRF24_CSN_PIN           GPIO_PIN_7

#define NRF24_SPI_BASE          SSI0_BASE

//-----------------------------------------------------------------

extern volatile unsigned long sysTickCount;

//-----------------------------------------------------------------

void nRF24_config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);

    SSIConfigSetExpClk(NRF24_SPI_BASE, SysCtlClockGet(),
                       SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(NRF24_SPI_BASE);
}

//-----------------------------------------------------------------

void nRF24_ce(uint8_t mode)
{
    GPIOPinWrite(NRF24_CE_PORT_BASE, NRF24_CE_PIN, mode);
}

//-----------------------------------------------------------------

void nRF24_csn(uint8_t mode)
{
    GPIOPinWrite(NRF24_CSN_PORT_BASE, NRF24_CSN_PIN, mode);
}

//-----------------------------------------------------------------

void nRF24_delay(unsigned long us)
{
    delay(us);
}

//-----------------------------------------------------------------

void nRF24_setTimeout(unsigned long us)
{
    sysTickCount = us;
}

//-----------------------------------------------------------------

bool nRF24_checkTimeout()
{
    return !sysTickCount;
}

//-----------------------------------------------------------------

uint8_t nRF24_spiTransfer(uint8_t data)
{
    unsigned long result;

    SSIDataPut(NRF24_SPI_BASE, data);
    while (SSIBusy(SSI0_BASE));
    SSIDataGet(NRF24_SPI_BASE, &result);

    return (uint8_t) result;
}
