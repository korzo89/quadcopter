/*
 * nrf24l01.c
 *
 *  Created on: 29-10-2013
 *      Author: Korzo
 */

#include "nrf24l01.h"
#include <stdlib.h>

//-----------------------------------------------------------------

void nrf24_init(unsigned char config,
                unsigned char rxActive,
                unsigned char enAutoAck,
                unsigned char enRxAddr,
                unsigned char setupAW,
                unsigned char setupRetr,
                unsigned char rfCh,
                unsigned char rfSetup,
                unsigned char *rxAddrP0,
                unsigned char *rxAddrP1,
                unsigned char rxAddrP2,
                unsigned char rxAddrP3,
                unsigned char rxAddrP4,
                unsigned char rxAddrP5,
                unsigned char *txAddr,
                unsigned char rxPwP0,
                unsigned char rxPwP1,
                unsigned char rxPwP2,
                unsigned char rxPwP3,
                unsigned char rxPwP4,
                unsigned char rxPwP5)
{
    unsigned char data[5];

    nrf24_writeRegister(NRF24_EN_AA, &enAutoAck, 1);
    nrf24_writeRegister(NRF24_EN_RXADDR, &enRxAddr, 1);
    nrf24_writeRegister(NRF24_SETUP_AW, &setupAW, 1);
    nrf24_writeRegister(NRF24_SETUP_RETR, &setupRetr, 1);
    nrf24_writeRegister(NRF24_RF_CH, &rfCh, 1);
    nrf24_writeRegister(NRF24_RF_SETUP, &rfSetup, 1);

    if (rxAddrP0 != NULL)
        nrf24_setRxAddr(rxAddrP0, 5, 0);
    else
    {
        data[0] = NRF24_RX_ADDR_P0_B0_DEFAULT_VAL;
        data[1] = NRF24_RX_ADDR_P0_B1_DEFAULT_VAL;
        data[2] = NRF24_RX_ADDR_P0_B2_DEFAULT_VAL;
        data[3] = NRF24_RX_ADDR_P0_B3_DEFAULT_VAL;
        data[4] = NRF24_RX_ADDR_P0_B4_DEFAULT_VAL;

        nrf24_setRxAddr(data, 5, 0);
    }

    if (rxAddrP1 != NULL)
        nrf24_setRxAddr(rxAddrP1, 5, 1);
    else
    {
        data[0] = NRF24_RX_ADDR_P1_B0_DEFAULT_VAL;
        data[1] = NRF24_RX_ADDR_P1_B1_DEFAULT_VAL;
        data[2] = NRF24_RX_ADDR_P1_B2_DEFAULT_VAL;
        data[3] = NRF24_RX_ADDR_P1_B3_DEFAULT_VAL;
        data[4] = NRF24_RX_ADDR_P1_B4_DEFAULT_VAL;

        nrf24_setRxAddr(data, 5, 1);
    }

    nrf24_setRxAddr(&rxAddrP2, 1, 2);
    nrf24_setRxAddr(&rxAddrP3, 1, 3);
    nrf24_setRxAddr(&rxAddrP4, 1, 4);
    nrf24_setRxAddr(&rxAddrP5, 1, 5);

    if (txAddr != NULL)
        nrf24_setTxAddr(txAddr, 5);
    else
    {
        data[0] = NRF24_TX_ADDR_B0_DEFAULT_VAL;
        data[1] = NRF24_TX_ADDR_B1_DEFAULT_VAL;
        data[2] = NRF24_TX_ADDR_B2_DEFAULT_VAL;
        data[3] = NRF24_TX_ADDR_B3_DEFAULT_VAL;
        data[4] = NRF24_TX_ADDR_B4_DEFAULT_VAL;

        nrf24_setTxAddr(data, 5);
    }

    nrf24_writeRegister(NRF24_RX_PW_P0, &rxPwP0, 1);
    nrf24_writeRegister(NRF24_RX_PW_P1, &rxPwP1, 1);
    nrf24_writeRegister(NRF24_RX_PW_P2, &rxPwP2, 1);
    nrf24_writeRegister(NRF24_RX_PW_P3, &rxPwP3, 1);
    nrf24_writeRegister(NRF24_RX_PW_P4, &rxPwP4, 1);
    nrf24_writeRegister(NRF24_RX_PW_P5, &rxPwP5, 1);

    if (config & NRF24_CONFIG_PWR_UP)
        nrf24_powerUpParam(rxActive, config);
    else
        nrf24_powerDownParam(config);

    nrf24_clearFlush();
}

//-----------------------------------------------------------------

void nrf24_initMinimal(bool rxActive, unsigned char rxPwP0, bool enAutoAck)
{
    unsigned char config;

    config = NRF24_CONFIG_DEFAULT_VAL | NRF24_CONFIG_PWR_UP;
    if (rxActive)
        config |= NRF24_CONFIG_PRIM_RX;

    nrf24_init(config,
               true,
               enAutoAck ? NRF24_EN_AA_ENAA_P0 : NRF24_EN_AA_ENAA_NONE,
               NRF24_EN_RXADDR_DEFAULT_VAL,
               NRF24_SETUP_AW_DEFAULT_VAL,
               NRF24_SETUP_RETR_DEFAULT_VAL,
               NRF24_RF_CH_DEFAULT_VAL,
               NRF24_RF_SETUP_DEFAULT_VAL,
               NULL,
               NULL,
               NRF24_RX_ADDR_P2_DEFAULT_VAL,
               NRF24_RX_ADDR_P3_DEFAULT_VAL,
               NRF24_RX_ADDR_P4_DEFAULT_VAL,
               NRF24_RX_ADDR_P5_DEFAULT_VAL,
               NULL,
               rxPwP0,
               NRF24_RX_PW_P1_DEFAULT_VAL,
               NRF24_RX_PW_P2_DEFAULT_VAL,
               NRF24_RX_PW_P3_DEFAULT_VAL,
               NRF24_RX_PW_P4_DEFAULT_VAL,
               NRF24_RX_PW_P5_DEFAULT_VAL);
}

//-----------------------------------------------------------------

void nrf24_powerUp(bool rxActive)
{
    unsigned char config;

    nrf24_readRegister(NRF24_CONFIG, &config, 1);

    if (config & NRF24_CONFIG_PWR_UP)
        return;

    nrf24_powerUpParam(rxActive, config);
}

//-----------------------------------------------------------------

void nrf24_powerUpParam(bool rxActive, unsigned char config)
{
    config |= NRF24_CONFIG_PWR_UP;

    nrf24_writeRegister(NRF24_CONFIG, &config, 1);

    nrf24_delay(1);

    if (!(config & NRF24_CONFIG_PRIM_RX))
        nrf24_clearCE();
    else
    {
        if (rxActive)
            nrf24_setCE();
        else
            nrf24_clearCE();
    }
}

//-----------------------------------------------------------------

void nrf24_powerDown(void)
{
    unsigned char config;

    nrf24_readRegister(NRF24_CONFIG, &config, 1);

    if (!(config & NRF24_CONFIG_PWR_UP))
        return;

    nrf24_powerDownParam(config);
}

//-----------------------------------------------------------------

void nrf24_powerDownParam(unsigned char config)
{
    config &= ~NRF24_CONFIG_PWR_UP;

    nrf24_writeRegister(NRF24_CONFIG, &config, 1);

    nrf24_clearCE();
}

//-----------------------------------------------------------------

unsigned char nrf24_writeRegister(unsigned char reg, unsigned char *data, unsigned int len)
{
    return nrf24_execCmd(NRF24_W_REGISTER | (reg & NRF24_W_REGISTER_DATA), data, len, false);
}

//-----------------------------------------------------------------

unsigned char nrf24_readRegister(unsigned char reg, unsigned char *data, unsigned int len)
{
    return nrf24_execCmd(reg & NRF24_W_REGISTER_DATA, data, len, true);
}

//-----------------------------------------------------------------

unsigned char nrf24_writeTxPayload(unsigned char *data, unsigned int len, bool transmit)
{
    unsigned char status;

    status = nrf24_execCmd(NRF24_W_TX_PAYLOAD, data, len, false);

    if (transmit)
        nrf24_transmit();

    return status;
}

//-----------------------------------------------------------------

unsigned char nrf24_readRxPayload(unsigned char *data, unsigned int len)
{
    unsigned char status;

    nrf24_clearCE();
    status = nrf24_execCmd(NRF24_R_RX_PAYLOAD, data, len, true);
    nrf24_setCE();

    return status;
}

//-----------------------------------------------------------------

unsigned char nrf24_flushTx(void)
{
    return nrf24_execCmd(NRF24_FLUSH_TX, NULL, 0, false);
}

//-----------------------------------------------------------------

unsigned char nrf24_flushRx(void)
{
    return nrf24_execCmd(NRF24_FLUSH_RX, NULL, 0, false);
}

//-----------------------------------------------------------------

unsigned char nrf24_nop(void)
{
    return nrf24_execCmd(NRF24_NOP, NULL, 0, false);
}

//-----------------------------------------------------------------

void nrf24_setAsRx(bool rxActive)
{
    unsigned char config;

    nrf24_readRegister(NRF24_CONFIG, &config, 1);

    if (config & NRF24_CONFIG_PRIM_RX)
        return;

    config |= NRF24_CONFIG_PRIM_RX;

    nrf24_writeRegister(NRF24_CONFIG, &config, 1);

    if (rxActive)
        nrf24_setCE();
    else
        nrf24_clearCE();
}

//-----------------------------------------------------------------

void nrf24_setAsRxParam(bool rxActive, unsigned char config)
{
    config |= NRF24_CONFIG_PRIM_RX;

    if (config & NRF24_CONFIG_PWR_UP)
        nrf24_powerUpParam(rxActive, config);
    else
        nrf24_powerDownParam(config);
}

//-----------------------------------------------------------------

void nrf24_setAsTx(void)
{
    unsigned char config;

    nrf24_readRegister(NRF24_CONFIG, &config, 1);

    if (!(config & NRF24_CONFIG_PRIM_RX))
        return;

    config &= ~NRF24_CONFIG_PRIM_RX;

    nrf24_writeRegister(NRF24_CONFIG, &config, 1);

    nrf24_clearCE();
}

//-----------------------------------------------------------------

void nrf24_setAsTxParam(unsigned char config)
{
    config &= ~NRF24_CONFIG_PRIM_RX;

    if (config & NRF24_CONFIG_PWR_UP)
        nrf24_powerUpParam(false, config);
    else
        nrf24_powerDownParam(config);
}

//-----------------------------------------------------------------

unsigned char nrf24_getConfig(void)
{
    unsigned char config;

    nrf24_readRegister(NRF24_CONFIG, &config, 1);

    return config;
}

//-----------------------------------------------------------------

void nrf24_setConfig(unsigned char config)
{
    nrf24_writeRegister(NRF24_CONFIG, &config, 1);
}

//-----------------------------------------------------------------

unsigned char nrf24_getRFChannel(void)
{
    unsigned char channel;

    nrf24_readRegister(NRF24_RF_CH, &channel, 1);

    return channel;
}

//-----------------------------------------------------------------

void nrf24_setRFChannel(unsigned char channel)
{
    channel &= ~NRF24_RF_CH_RESERVED;

    nrf24_writeRegister(NRF24_RF_CH, &channel, 1);
}

//-----------------------------------------------------------------

unsigned char nrf24_getStatus(void)
{
    return nrf24_nop();
}

//-----------------------------------------------------------------

unsigned char nrf24_getObserveTx(void)
{
    unsigned char data;

    nrf24_readRegister(NRF24_OBSERVE_TX, &data, 1);

    return data;
}

//-----------------------------------------------------------------

void nrf24_setRxAddr(unsigned char *addr, unsigned int len, unsigned char pipe)
{
    if (pipe > 5)
        return;

    nrf24_writeRegister(NRF24_RX_ADDR_P0 + pipe, addr, len);
}

//-----------------------------------------------------------------

void nrf24_setTxAddr(unsigned char *addr, unsigned int len)
{
    nrf24_writeRegister(NRF24_TX_ADDR, addr, len);
}

//-----------------------------------------------------------------

void nrf24_setPayloadWidth(unsigned char width, unsigned char pipe)
{
    if (pipe > 5 || width > 32)
        return;

    nrf24_writeRegister(NRF24_RX_PW_P0 + pipe, &width, 1);
}

//-----------------------------------------------------------------

unsigned char nrf24_getPayloadWidth(unsigned char pipe)
{
    unsigned char width;

    if (pipe > 5)
        return 0;

    nrf24_readRegister(NRF24_RX_PW_P0 + pipe, &width, 1);

    return width;
}

//-----------------------------------------------------------------

unsigned char nrf24_getFIFOStatus(void)
{
    unsigned char data;

    nrf24_readRegister(NRF24_FIFO_STATUS, &data, 1);

    return data;
}

//-----------------------------------------------------------------

bool nrf24_autoAckEnabled(unsigned char pipe)
{
    unsigned char data;

    if (pipe > 5)
        return false;

    nrf24_readRegister(NRF24_EN_AA, &data, 1);

    return data & (1 << pipe);
}

//-----------------------------------------------------------------

void nrf24_autoAckEnable(unsigned char pipe)
{
    unsigned char data;

    if (pipe > 5)
        return;

    nrf24_readRegister(NRF24_EN_AA, &data, 1);

    if (data & (1 << pipe))
        return;

    data |= 1 << pipe;

    nrf24_writeRegister(NRF24_EN_AA, &data, 1);
}

//-----------------------------------------------------------------

void nrf24_autoAckDisable(unsigned char pipe)
{
    unsigned char data;

    if (pipe > 5)
        return;

    nrf24_readRegister(NRF24_EN_AA, &data, 1);

    if (!(data & (1 << pipe)))
        return;

    data &= ~(1 << pipe);

    nrf24_writeRegister(NRF24_EN_AA, &data, 1);
}

//-----------------------------------------------------------------

bool nrf24_pipeEnabled(unsigned char pipe)
{
    unsigned char data;

    if (pipe > 5)
        return false;

    nrf24_readRegister(NRF24_EN_RXADDR, &data, 1);

    return data & (1 << pipe);
}

//-----------------------------------------------------------------

void nrf24_pipeEnable(unsigned char pipe)
{
    unsigned char data;

    if (pipe > 5)
        return;

    nrf24_readRegister(NRF24_EN_RXADDR, &data, 1);

    if (data & (1 << pipe))
        return;

    data |= 1 << pipe;

    nrf24_writeRegister(NRF24_EN_RXADDR, &data, 1);
}

//-----------------------------------------------------------------

void nrf24_pipeDisable(unsigned char pipe)
{
    unsigned char data;

    if (pipe > 5)
        return;

    nrf24_readRegister(NRF24_EN_RXADDR, &data, 1);

    if (!(data & (1 << pipe)))
        return;

    data &= ~(1 << pipe);

    nrf24_writeRegister(NRF24_EN_RXADDR, &data, 1);
}

//-----------------------------------------------------------------

bool nrf24_carrierDetect(void)
{
    unsigned char data;

    nrf24_readRegister(NRF24_CD, &data, 1);

    return data;
}

//-----------------------------------------------------------------

void nrf24_clearFlush(void)
{
    nrf24_flushRx();
    nrf24_flushTx();
    nrf24_irqClearAll();
}

//-----------------------------------------------------------------

unsigned char nrf24_getRxPipe(void)
{
    return nrf24_getRxPipeFromStatus(nrf24_getStatus());
}

//-----------------------------------------------------------------

unsigned char nrf24_getRxPipeFromStatus(unsigned char status)
{
    return (status & 0x0E) >> 1;
}

//-----------------------------------------------------------------

bool nrf24_irq_RX_DR(void)
{
    return nrf24_getStatus() & NRF24_STATUS_RX_DR;
}

//-----------------------------------------------------------------

bool nrf24_irq_TX_DS(void)
{
    return nrf24_getStatus() & NRF24_STATUS_TX_DS;
}

//-----------------------------------------------------------------

bool nrf24_irq_MAX_RT(void)
{
    return nrf24_getStatus() & NRF24_STATUS_MAX_RT;
}

//-----------------------------------------------------------------

void nrf24_irqClearAll(void)
{
    unsigned char data = NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT;

    nrf24_writeRegister(NRF24_STATUS, &data, 1);
}

//-----------------------------------------------------------------

void nrf24_irqClear_RX_DR(void)
{
    unsigned char data = NRF24_STATUS_RX_DR;

    nrf24_writeRegister(NRF24_STATUS, &data, 1);
}

//-----------------------------------------------------------------

void nrf24_irqClear_TX_DS(void)
{
    unsigned char data = NRF24_STATUS_TX_DS;

    nrf24_writeRegister(NRF24_STATUS, &data, 1);
}

//-----------------------------------------------------------------

void nrf24_irqClear_MAX_RT(void)
{
    unsigned char data = NRF24_STATUS_MAX_RT;

    nrf24_writeRegister(NRF24_STATUS, &data, 1);
}

//-----------------------------------------------------------------

void nrf24_transmit(void)
{
    nrf24_setCE();
    nrf24_delay(1);
    nrf24_clearCE();
}

//-----------------------------------------------------------------

unsigned char nrf24_execCmd(unsigned char cmd, unsigned char *data, unsigned int len, bool read)
{
    unsigned int i;
    unsigned char status, temp;

    nrf24_clearCSN();

    status = nrf24_spiSendByte(cmd);
    for (i = 0; i < len; i++)
    {
        temp = nrf24_spiSendByte(data[i]);
        if (read)
            data[i] = temp;
    }

    nrf24_setCSN();

    return status;
}
