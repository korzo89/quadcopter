/*
 * nrf24l01.c
 *
 *  Created on: 29-10-2013
 *      Author: Korzo
 */

#include "nrf24l01.h"

#include <utils/delay.h>

#include <stellaris_config.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stdlib.h>

//-----------------------------------------------------------------

#define NRF_CE_PORT           GPIO_PORTA_BASE
#define NRF_CE_PIN            GPIO_PIN_3

#define NRF_CSN_PORT          GPIO_PORTA_BASE
#define NRF_CSN_PIN           GPIO_PIN_7

#define NRF_IRQ_PORT          GPIO_PORTC_BASE
#define NRF_IRQ_PIN           GPIO_PIN_7

#define NRF_SPI_BASE          SSI0_BASE

#define CE_SET()              GPIOPinWrite(NRF_CE_PORT, NRF_CE_PIN, 0xFF)
#define CE_CLEAR()            GPIOPinWrite(NRF_CE_PORT, NRF_CE_PIN, 0x00)

#define CSN_SET()             GPIOPinWrite(NRF_CSN_PORT, NRF_CSN_PIN, 0xFF)
#define CSN_CLEAR()           GPIOPinWrite(NRF_CSN_PORT, NRF_CSN_PIN, 0x00)

#define NRF_DELAY_MS(x)       DELAY_MS(x)

//-----------------------------------------------------------------

static IRQCallback nrfIRQCallback = NULL;

//-----------------------------------------------------------------

void GPIOCIntHandler(void)
{
    unsigned long status = GPIOPinIntStatus(NRF_IRQ_PORT, true);
    GPIOPinIntClear(NRF_IRQ_PORT, status);

    if (status & NRF_IRQ_PIN && nrfIRQCallback)
        nrfIRQCallback();
}

//-----------------------------------------------------------------

void nrfInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    // config nRF pins
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);

    CSN_SET();
    CE_CLEAR();

    // config IRQ pin
    GPIOPinTypeGPIOInput(NRF_IRQ_PORT, NRF_IRQ_PIN);
    GPIOIntTypeSet(NRF_IRQ_PORT, NRF_IRQ_PIN, GPIO_LOW_LEVEL);
    GPIOPinIntEnable(NRF_IRQ_PORT, NRF_IRQ_PIN);

    IntPrioritySet(INT_GPIOC, configKERNEL_INTERRUPT_PRIORITY);
    IntEnable(INT_GPIOC);

    // config SPI
    SSIConfigSetExpClk(NRF_SPI_BASE, SysCtlClockGet(),
                       SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(NRF_SPI_BASE);
}

//-----------------------------------------------------------------

static uint8_t nrfSPISendByte(uint8_t data)
{
    unsigned long result;

    SSIDataPut(NRF_SPI_BASE, data);
    while (SSIBusy(NRF_SPI_BASE));
    SSIDataGet(NRF_SPI_BASE, &result);

    return (uint8_t)result;
}

//-----------------------------------------------------------------

uint8_t nrfExecCmd(uint8_t cmd, uint8_t *data, unsigned int len, bool read)
{
    unsigned int i;
    uint8_t status, temp;

    CSN_CLEAR();

    status = nrfSPISendByte(cmd);
    for (i = 0; i < len; i++)
    {
        temp = nrfSPISendByte(data[i]);
        if (read)
            data[i] = temp;
    }

    CSN_SET();

    return status;
}

//-----------------------------------------------------------------

uint8_t nrfWriteRegister(uint8_t reg, uint8_t *data, unsigned int len)
{
    return nrfExecCmd(NRF_W_REGISTER | (reg & NRF_W_REGISTER_MASK), data, len, false);
}

//-----------------------------------------------------------------

uint8_t nrfWriteRegisterByte(uint8_t reg, uint8_t data)
{
    return nrfWriteRegister(reg, &data, 1);
}

//-----------------------------------------------------------------

uint8_t nrfReadRegister(uint8_t reg, uint8_t *data, unsigned int len)
{
    return nrfExecCmd(NRF_R_REGISTER | (reg & NRF_R_REGISTER_MASK), data, len, true);
}

//-----------------------------------------------------------------

uint8_t nrfReadRegisterByte(uint8_t reg)
{
    uint8_t res;
    nrfReadRegister(reg, &res, 1);
    return res;
}

//-----------------------------------------------------------------

void nrfPowerUp(void)
{
    uint8_t config = nrfReadRegisterByte(NRF_CONFIG);
    if (config & NRF_CONFIG_PWR_UP)
        return;

    config |= NRF_CONFIG_PWR_UP;
    nrfWriteRegister(NRF_CONFIG, &config, 1);
}

//-----------------------------------------------------------------

uint8_t nrfWriteTxPayload(uint8_t *data, unsigned int len, bool transmit)
{
    uint8_t status = nrfExecCmd(NRF_W_TX_PAYLOAD, data, len, false);

    if (transmit)
        nrfTransmit();

    return status;
}

//-----------------------------------------------------------------

uint8_t nrfReadRxPayload(uint8_t *data, unsigned int len)
{
    uint8_t status;

    CE_CLEAR();
    status = nrfExecCmd(NRF_R_RX_PAYLOAD, data, len, true);
    CE_SET();

    return status;
}

//-----------------------------------------------------------------

uint8_t nrfFlushTx(void)
{
    return nrfExecCmd(NRF_FLUSH_TX, NULL, 0, false);
}

//-----------------------------------------------------------------

uint8_t nrfFlushRx(void)
{
    return nrfExecCmd(NRF_FLUSH_RX, NULL, 0, false);
}

//-----------------------------------------------------------------

uint8_t nrfNop(void)
{
    return nrfExecCmd(NRF_NOP, NULL, 0, false);
}

//-----------------------------------------------------------------

void nrfSetAsRx(void)
{
    uint8_t config = nrfReadRegisterByte(NRF_CONFIG);
    if (config & NRF_CONFIG_PRIM_RX)
        return;

    config |= NRF_CONFIG_PRIM_RX;
    nrfWriteRegisterByte(NRF_CONFIG, config);

    CE_SET();
}

//-----------------------------------------------------------------

void nrfSetAsTx(void)
{
    uint8_t config = nrfReadRegisterByte(NRF_CONFIG);
    if (!(config & NRF_CONFIG_PRIM_RX))
        return;

    config &= ~NRF_CONFIG_PRIM_RX;
    nrfWriteRegisterByte(NRF_CONFIG, config);

    CE_CLEAR();
}

//-----------------------------------------------------------------

void nrfSetConfig(uint8_t config)
{
    nrfWriteRegisterByte(NRF_CONFIG, config);
}

//-----------------------------------------------------------------

void nrfSetRFChannel(uint8_t channel)
{
    channel &= ~NRF_RF_CH_RESERVED;
    nrfWriteRegisterByte(NRF_RF_CH, channel);
}

//-----------------------------------------------------------------

uint8_t nrfGetStatus(void)
{
    return nrfNop();
}

//-----------------------------------------------------------------

uint8_t nrfGetObserveTx(void)
{
    return nrfReadRegisterByte(NRF_OBSERVE_TX);
}

//-----------------------------------------------------------------

void nrfSetRxAddr(uint8_t *addr, unsigned int len, uint8_t pipe)
{
    if (pipe > 5)
        return;

    nrfWriteRegister(NRF_RX_ADDR_P0 + pipe, addr, len);
}

//-----------------------------------------------------------------

void nrfSetTxAddr(uint8_t *addr, unsigned int len)
{
    nrfWriteRegister(NRF_TX_ADDR, addr, len);
}

//-----------------------------------------------------------------

void nrfSetPayloadWidth(uint8_t width, uint8_t pipe)
{
    if (pipe > 5 || width > 32)
        return;

    nrfWriteRegisterByte(NRF_RX_PW_P0 + pipe, width);
}

//-----------------------------------------------------------------

uint8_t nrfGetFIFOStatus(void)
{
    return nrfReadRegisterByte(NRF_FIFO_STATUS);
}

//-----------------------------------------------------------------

void nrfAutoAckEnable(uint8_t pipe)
{
    if (pipe > 5)
        return;

    uint8_t data = nrfReadRegisterByte(NRF_EN_AA);
    if (data & (1 << pipe))
        return;

    data |= 1 << pipe;
    nrfWriteRegisterByte(NRF_EN_AA, data);
}

//-----------------------------------------------------------------

void nrfPipeEnable(uint8_t pipe)
{
    if (pipe > 5)
        return;

    uint8_t data = nrfReadRegisterByte(NRF_EN_RXADDR);
    if (data & (1 << pipe))
        return;

    data |= 1 << pipe;
    nrfWriteRegisterByte(NRF_EN_RXADDR, data);
}

//-----------------------------------------------------------------

bool nrfCarrierDetect(void)
{
    return nrfReadRegisterByte(NRF_CD);
}

//-----------------------------------------------------------------

uint8_t nrfGetRxPipe(void)
{
    return (nrfGetStatus() & NRF_STATUS_RX_P_NO) >> 1;
}

//-----------------------------------------------------------------

void nrfClearIRQ(uint8_t irq)
{
    nrfWriteRegisterByte(NRF_STATUS, irq);
}

//-----------------------------------------------------------------

void nrfClearAllIRQ(void)
{
    nrfClearIRQ(NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
}

//-----------------------------------------------------------------

void nrfClearFlush(void)
{
    nrfClearAllIRQ();
    nrfFlushRx();
    nrfFlushTx();
}

//-----------------------------------------------------------------

void nrfTransmit(void)
{
    CE_SET();
    NRF_DELAY_MS(1);
    CE_CLEAR();
}

//-----------------------------------------------------------------

void nrfSetIRQCallback(IRQCallback callback)
{
    nrfIRQCallback = callback;
}
