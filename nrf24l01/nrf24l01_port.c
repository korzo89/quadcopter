/*
 * nrf24l01_port.c
 *
 *  Created on: 30-10-2013
 *      Author: Korzo
 */

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "../utils/delay.h"

#include "nrf24l01.h"

//-----------------------------------------------------------------

#define NRF24_CE_PORT_BASE      GPIO_PORTA_BASE
#define NRF24_CE_PIN            GPIO_PIN_3

#define NRF24_CSN_PORT_BASE     GPIO_PORTA_BASE
#define NRF24_CSN_PIN           GPIO_PIN_7

#define NRF24_IRQ_PORT_BASE     GPIO_PORTC_BASE
#define NRF24_IRQ_PIN           GPIO_PIN_7

#define NRF24_SPI_BASE          SSI0_BASE

//-----------------------------------------------------------------

void nrf24_config(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);

    GPIOPinTypeGPIOInput(NRF24_IRQ_PORT_BASE, NRF24_IRQ_PIN);

    SSIConfigSetExpClk(NRF24_SPI_BASE, SysCtlClockGet(),
                       SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(NRF24_SPI_BASE);
}

//-----------------------------------------------------------------

void nrf24_delay(unsigned int ms)
{
    delay(ms);
}

//-----------------------------------------------------------------

bool nrf24_irqPinActive(void)
{
    return ~GPIOPinRead(NRF24_IRQ_PORT_BASE, NRF24_IRQ_PIN);
}

//-----------------------------------------------------------------

void nrf24_clearCE(void)
{
    GPIOPinWrite(NRF24_CE_PORT_BASE, NRF24_CE_PIN, 0x00);
}

//-----------------------------------------------------------------

void nrf24_setCE(void)
{
    GPIOPinWrite(NRF24_CE_PORT_BASE, NRF24_CE_PIN, 0xFF);
}

//-----------------------------------------------------------------

void nrf24_clearCSN(void)
{
    GPIOPinWrite(NRF24_CSN_PORT_BASE, NRF24_CSN_PIN, 0x00);
}

//-----------------------------------------------------------------

void nrf24_setCSN(void)
{
    GPIOPinWrite(NRF24_CSN_PORT_BASE, NRF24_CSN_PIN, 0xFF);
}

//-----------------------------------------------------------------

unsigned char nrf24_spiSendByte(unsigned char data)
{
    unsigned long result;

    SSIDataPut(NRF24_SPI_BASE, data);
    while (SSIBusy(NRF24_SPI_BASE));
    SSIDataGet(NRF24_SPI_BASE, &result);

    return (unsigned char)result;
}
