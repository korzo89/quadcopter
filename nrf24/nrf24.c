/*
 * nrf24.c
 *
 *  Created on: 18-08-2013
 *      Author: Korzo
 */

#include "nrf24.h"

//-----------------------------------------------------------------

static uint8_t configuration;

//-----------------------------------------------------------------

uint8_t nRF24_spiCommand(uint8_t cmd)
{
    uint8_t status;

    nRF24_csn(NRF24_PIN_LOW);
    status = nRF24_spiTransfer(cmd);
    nRF24_csn(NRF24_PIN_HIGH);

    return status;
}

//-----------------------------------------------------------------

uint8_t nRF24_spiRead(uint8_t cmd)
{
    uint8_t status;

    nRF24_csn(NRF24_PIN_LOW);
    nRF24_spiTransfer(cmd);
    status = nRF24_spiTransfer(NRF24_DUMMY_BYTE);
    nRF24_csn(NRF24_PIN_HIGH);

    return status;
}

//-----------------------------------------------------------------

uint8_t nRF24_spiReadBurst(uint8_t cmd, uint8_t* dest, uint8_t len)
{
    uint8_t status;

    nRF24_csn(NRF24_PIN_LOW);

    status = nRF24_spiTransfer(cmd);
    while (len--)
        *dest++ = nRF24_spiTransfer(NRF24_DUMMY_BYTE);

    nRF24_csn(NRF24_PIN_HIGH);

    return status;
}

//-----------------------------------------------------------------

uint8_t nRF24_spiWrite(uint8_t cmd, uint8_t val)
{
    uint8_t status;

    nRF24_csn(NRF24_PIN_LOW);
    status = nRF24_spiTransfer(cmd);
    nRF24_spiTransfer(val);
    nRF24_csn(NRF24_PIN_HIGH);

    return status;
}

//-----------------------------------------------------------------

uint8_t nRF24_spiWriteBurst(uint8_t cmd, uint8_t* src, uint8_t len)
{
    uint8_t status;

    nRF24_csn(NRF24_PIN_LOW);

    status = nRF24_spiTransfer(cmd);
    while (len--)
    	nRF24_spiTransfer(*src++);

    nRF24_csn(NRF24_PIN_HIGH);

    return status;
}

//-----------------------------------------------------------------

uint8_t nRF24_readRegister(uint8_t reg)
{
   return nRF24_spiRead((reg & NRF24_REGISTER_MASK) | NRF24_CMD_R_REGISTER);
}

//-----------------------------------------------------------------

uint8_t nRF24_readRegisterBurst(uint8_t reg, uint8_t* dest, uint8_t len)
{
    return nRF24_spiReadBurst((reg & NRF24_REGISTER_MASK) | NRF24_CMD_R_REGISTER, dest, len);
}

//-----------------------------------------------------------------

uint8_t nRF24_writeRegister(uint8_t reg, uint8_t val)
{
    return nRF24_spiWrite((reg & NRF24_REGISTER_MASK) | NRF24_CMD_W_REGISTER, val);
}

//-----------------------------------------------------------------

uint8_t nRF24_writeRegisterBurst(uint8_t reg, uint8_t* src, uint8_t len)
{
    return nRF24_spiWriteBurst((reg & NRF24_REGISTER_MASK) | NRF24_CMD_W_REGISTER, src, len);
}

//-----------------------------------------------------------------

void nRF24_init()
{
    nRF24_writeRegister(NRF24_REG_07_STATUS, NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);

    nRF24_powerDown();

    nRF24_flushTx();
    nRF24_flushRx();

    nRF24_powerUpRx();
}

//-----------------------------------------------------------------

uint8_t nRF24_getStatus()
{
    return nRF24_readRegister(NRF24_REG_07_STATUS);
}

//-----------------------------------------------------------------

uint8_t nRF24_flushTx()
{
    return nRF24_spiCommand(NRF24_CMD_FLUSH_TX);
}

//-----------------------------------------------------------------

uint8_t nRF24_flushRx()
{
    return nRF24_spiCommand(NRF24_CMD_FLUSH_RX);
}

//-----------------------------------------------------------------

void nRF24_setChannel(uint8_t channel)
{
    nRF24_writeRegister(NRF24_REG_05_RF_CH, channel & NRF24_RF_CH);
}

//-----------------------------------------------------------------

void nRF24_setConfiguration(uint8_t config)
{
    configuration = config;
}

//-----------------------------------------------------------------

void nRF24_setPipeAddress(uint8_t pipe, uint8_t* addr, uint8_t len)
{
    nRF24_writeRegisterBurst(NRF24_REG_0A_RX_ADDR_P0 + pipe, addr, len);
}

//-----------------------------------------------------------------

void nRF24_setRxAddress(uint8_t* addr)
{
    nRF24_setPipeAddress(0, addr, 5);
}

//-----------------------------------------------------------------

void nRF24_setTxAddress(uint8_t* addr)
{
    nRF24_writeRegisterBurst(NRF24_REG_0A_RX_ADDR_P0, addr, 5);
    nRF24_writeRegisterBurst(NRF24_REG_10_TX_ADDR, addr, 5);
}

//-----------------------------------------------------------------

void nRF24_setRetry(uint8_t delay, uint8_t count)
{
    nRF24_writeRegister(NRF24_REG_04_SETUP_RETR,
                        ((delay << 4) & NRF24_ARD) | (count & NRF24_ARC));
}

//-----------------------------------------------------------------

void nRF24_setPayloadSize(uint8_t size)
{
    nRF24_writeRegister(NRF24_REG_11_RX_PW_P0, size);
    nRF24_writeRegister(NRF24_REG_12_RX_PW_P1, size);
}

//-----------------------------------------------------------------

void nRF24_setRF(NRF24DataRate rate, NRF24TransmitPower power)
{
    uint8_t val = ((uint8_t) power << 1) & NRF24_PWR;

    if (rate == NRF24DataRate250kbps)
        val |= NRF24_RF_DR_LOW;
    else if (rate == NRF24DataRate2Mbps)
        val |= NRF24_RF_DR_HIGH;

    nRF24_writeRegister(NRF24_REG_06_RF_SETUP, val);
}

//-----------------------------------------------------------------

void nRF24_powerDown()
{
    nRF24_writeRegister(NRF24_REG_00_CONFIG, configuration);
    nRF24_ce(NRF24_PIN_LOW);
}

//-----------------------------------------------------------------

void nRF24_powerUpRx()
{
    nRF24_writeRegister(NRF24_REG_00_CONFIG, configuration | NRF24_PWR_UP | NRF24_PRIM_RX);
    nRF24_ce(NRF24_PIN_HIGH);
    nRF24_delay(130);
}

//-----------------------------------------------------------------

void nRF24_powerUpTx()
{
    nRF24_ce(NRF24_PIN_LOW);
    nRF24_writeRegister(NRF24_REG_00_CONFIG, configuration | NRF24_PWR_UP);
    nRF24_ce(NRF24_PIN_HIGH);
    nRF24_delay(130);
}

//-----------------------------------------------------------------

void nRF24_send(uint8_t* data, uint8_t len, bool noAck)
{
    nRF24_powerUpTx();
    nRF24_spiWriteBurst(noAck ? NRF24_CMD_W_TX_PAYLOAD_NOACK : NRF24_CMD_W_TX_PAYLOAD,
                        data, len);
}

//-----------------------------------------------------------------

bool nRF24_waitPacketSent()
{
    uint8_t status;

    if (nRF24_readRegister(NRF24_REG_00_CONFIG) & NRF24_PRIM_RX)
        return false;

    while (!((status = nRF24_getStatus()) & (NRF24_TX_DS | NRF24_MAX_RT)));

    nRF24_writeRegister(NRF24_REG_07_STATUS, NRF24_TX_DS | NRF24_MAX_RT);
    if (status & NRF24_MAX_RT)
        nRF24_flushTx();

    return status & NRF24_TX_DS;
}

//-----------------------------------------------------------------

bool nRF24_isSending()
{
    return !(nRF24_readRegister(NRF24_REG_00_CONFIG) & NRF24_PRIM_RX) &&
            !(nRF24_getStatus() & (NRF24_TX_DS | NRF24_MAX_RT));
}

//-----------------------------------------------------------------

bool nRF24_available()
{
    if (nRF24_readRegister(NRF24_REG_17_FIFO_STATUS) & NRF24_RX_EMPTY)
        return false;

    if (nRF24_spiRead(NRF24_CMD_R_RX_PL_WID) > 32)
    {
        nRF24_flushRx();
        return false;
    }

    return true;
}

//-----------------------------------------------------------------

void nRF24_waitAvailable()
{
    nRF24_powerUpRx();
    while (!nRF24_available());
}

//-----------------------------------------------------------------

bool nRF24_waitAvailableTimeout(unsigned long us)
{
    nRF24_powerUpRx();
    nRF24_setTimeout(us);
    while (!nRF24_checkTimeout())
    {
        if (nRF24_available())
            return true;
    }

    return false;
}

//-----------------------------------------------------------------

bool nRF24_receive(uint8_t* buf, uint8_t* len)
{
    nRF24_writeRegister(NRF24_REG_07_STATUS, NRF24_RX_DR);

    if (!nRF24_available())
        return false;

    *len = nRF24_spiRead(NRF24_CMD_R_RX_PL_WID);

    nRF24_spiReadBurst(NRF24_CMD_R_RX_PAYLOAD, buf, *len);

    return true;
}
