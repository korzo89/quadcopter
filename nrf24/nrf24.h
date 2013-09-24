/*
 * nrf24.h
 *
 *  Created on: 18-08-2013
 *      Author: Korzo
 */

#ifndef NRF24_H_
#define NRF24_H_

//-----------------------------------------------------------------

#include "nrf24_defs.h"

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------

typedef enum {
    NRF24DataRate1Mbps = 0,
    NRF24DataRate2Mbps,
    NRF24DataRate250kbps
} NRF24DataRate;

typedef enum {
    NRF24TransmitPowerm18dBm = 0,
    NRF24TransmitPowerm12dBm,
    NRF24TransmitPowerm6dBm,
    NRF24TransmitPower0dBm
} NRF24TransmitPower;

//-----------------------------------------------------------------

void nRF24_config();

void nRF24_ce(uint8_t mode);
void nRF24_csn(uint8_t mode);

void nRF24_delay(unsigned long us);
void nRF24_setTimeout(unsigned long us);
bool nRF24_checkTimeout();

uint8_t nRF24_spiTransfer(uint8_t data);

uint8_t nRF24_spiCommand(uint8_t cmd);
uint8_t nRF24_spiRead(uint8_t cmd);
uint8_t nRF24_spiReadBurst(uint8_t cmd, uint8_t* dest, uint8_t len);
uint8_t nRF24_spiWrite(uint8_t cmd, uint8_t val);
uint8_t nRF24_spiWriteBurst(uint8_t cmd, uint8_t* src, uint8_t len);

uint8_t nRF24_readRegister(uint8_t reg);
uint8_t nRF24_readRegisterBurst(uint8_t reg, uint8_t* dest, uint8_t len);
uint8_t nRF24_writeRegister(uint8_t reg, uint8_t val);
uint8_t nRF24_writeRegisterBurst(uint8_t reg, uint8_t* src, uint8_t len);

void nRF24_init();

uint8_t nRF24_getStatus();

uint8_t nRF24_flushTx();
uint8_t nRF24_flushRx();

void nRF24_setChannel(uint8_t channel);
void nRF24_setConfiguration(uint8_t config);
void nRF24_setPipeAddress(uint8_t pipe, uint8_t* addr, uint8_t len);
void nRF24_setRxAddress(uint8_t* addr);
void nRF24_setTxAddress(uint8_t* addr);
void nRF24_setRetry(uint8_t delay, uint8_t count);
void nRF24_setPayloadSize(uint8_t size);
void nRF24_setRF(NRF24DataRate rate, NRF24TransmitPower power);

void nRF24_powerDown();
void nRF24_powerUpRx();
void nRF24_powerUpTx();

void nRF24_send(uint8_t* data, uint8_t len, bool noAck);
bool nRF24_waitPacketSent();
bool nRF24_isSending();

bool nRF24_available();
void nRF24_waitAvailable();
bool nRF24_waitAvailableTimeout(unsigned long us);

bool nRF24_receive(uint8_t* buf, uint8_t* len);

//-----------------------------------------------------------------

#endif /* NRF24_H_ */
