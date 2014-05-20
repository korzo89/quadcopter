/*
 * nrf24l01.h
 *
 *  Created on: 29-10-2013
 *      Author: Korzo
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

#include "nrf24l01_defs.h"

//-----------------------------------------------------------------

#define NRF_CHECK_STATUS(x)     ( nrfGetStatus() & (x) )
#define NRF_CHECK_FIFO(x)		( nrfGetFIFOStatus() & (x) )

//-----------------------------------------------------------------

typedef void (*IRQCallback)(void);

//-----------------------------------------------------------------

void nrfInit(void);

uint8_t nrfExecCmd(uint8_t cmd, uint8_t *data, unsigned int len, bool read);

uint8_t nrfWriteRegister(uint8_t reg, uint8_t *data, unsigned int len);
uint8_t nrfWriteRegisterByte(uint8_t reg, uint8_t data);

uint8_t nrfReadRegister(uint8_t reg, uint8_t *data, unsigned int len);
uint8_t nrfReadRegisterByte(uint8_t reg);

void nrfPowerUp(void);

uint8_t nrfWriteTxPayload(uint8_t *data, unsigned int len, bool transmit);
uint8_t nrfReadRxPayload(uint8_t *data, unsigned int len);
uint8_t nrfFlushTx(void);
uint8_t nrfFlushRx(void);
uint8_t nrfNop(void);

void nrfSetAsRx(void);
void nrfSetAsTx(void);

void nrfSetConfig(uint8_t config);
void nrfSetRFChannel(uint8_t channel);
uint8_t nrfGetStatus(void);
uint8_t nrfGetObserveTx(void);
void nrfSetRxAddr(uint8_t *addr, unsigned int len, uint8_t pipe);
void nrfSetTxAddr(uint8_t *addr, unsigned int len);
void nrfSetPayloadWidth(uint8_t width, uint8_t pipe);
uint8_t nrfGetFIFOStatus(void);

void nrfAutoAckEnable(uint8_t pipe);
void nrfPipeEnable(uint8_t pipe);

bool nrfCarrierDetect(void);
uint8_t nrfGetRxPipe(void);

void nrfClearIRQ(uint8_t irq);
void nrfClearAllIRQ(void);
void nrfClearFlush(void);

void nrfTransmit(void);

bool nrfIsIRQActive(void);
void nrfSetIRQCallback(IRQCallback callback);

//-----------------------------------------------------------------

#endif /* NRF24L01_H_ */
