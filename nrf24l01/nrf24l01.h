/*
 * nrf24l01.h
 *
 *  Created on: 29-10-2013
 *      Author: Korzo
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_

//-----------------------------------------------------------------

#include <stdbool.h>

#include "nrf24l01_defs.h"

//-----------------------------------------------------------------

void nrf24_config(void);

void nrf24_delay(unsigned int ms);

bool nrf24_irqPinActive(void);
void nrf24_clearCE(void);
void nrf24_setCE(void);
void nrf24_clearCSN(void);
void nrf24_setCSN(void);

unsigned char nrf24_spiSendByte(unsigned char data);

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
                unsigned char rxPwP5);
void nrf24_initMinimal(bool rxActive, unsigned char rxPwP0, bool enAutoAck);

void nrf24_powerUp(bool rxActive);
void nrf24_powerUpParam(bool rxActive, unsigned char config);
void nrf24_powerDown(void);
void nrf24_powerDownParam(unsigned char config);

unsigned char nrf24_writeRegister(unsigned char reg, unsigned char *data, unsigned int len);
unsigned char nrf24_readRegister(unsigned char reg, unsigned char *data, unsigned int len);

unsigned char nrf24_writeTxPayload(unsigned char *data, unsigned int len, bool transmit);
unsigned char nrf24_readRxPayload(unsigned char *data, unsigned int len);
unsigned char nrf24_flushTx(void);
unsigned char nrf24_flushRx(void);
unsigned char nrf24_nop(void);

void nrf24_setAsRx(bool rx);
void nrf24_setAsRxParam(bool rx, unsigned char config);
void nrf24_setAsTx(void);
void nrf24_setAsTxParam(unsigned char config);

unsigned char nrf24_getConfig(void);
void nrf24_setConfig(unsigned char config);
unsigned char nrf24_getRFChannel(void);
void nrf24_setRFChannel(unsigned char channel);
unsigned char nrf24_getStatus(void);
unsigned char nrf24_getObserveTx(void);
void nrf24_setRxAddr(unsigned char *addr, unsigned int len, unsigned char pipe);
void nrf24_setTxAddr(unsigned char *addr, unsigned int len);
void nrf24_setPayloadWidth(unsigned char width, unsigned char pipe);
unsigned char nrf24_getPayloadWidth(unsigned char pipe);
unsigned char nrf24_getFIFOStatus(void);

bool nrf24_autoAckEnabled(unsigned char pipe);
void nrf24_autoAckEnable(unsigned char pipe);
void nrf24_autoAckDisable(unsigned char pipe);
bool nrf24_pipeEnabled(unsigned char pipe);
void nrf24_pipeEnable(unsigned char pipe);
void nrf24_pipeDisable(unsigned char pipe);

bool nrf24_carrierDetect(void);
void nrf24_clearFlush(void);
unsigned char nrf24_getRxPipe(void);
unsigned char nrf24_getRxPipeFromStatus(unsigned char status);

bool nrf24_irq_RX_DR(void);
bool nrf24_irq_TX_DS(void);
bool nrf24_irq_MAX_RT(void);
void nrf24_irqClearAll(void);
void nrf24_irqClear_RX_DR(void);
void nrf24_irqClear_TX_DS(void);
void nrf24_irqClear_MAX_RT(void);

void nrf24_transmit(void);

unsigned char nrf24_execCmd(unsigned char cmd, unsigned char *data, unsigned int len, bool read);
void nrf24_spiTransfer(unsigned char *data, unsigned int len, bool read);

//-----------------------------------------------------------------

#endif /* NRF24L01_H_ */
