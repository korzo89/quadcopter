/*
 * comm.c
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#include "comm.h"
#include <drivers/nrf24l01.h>
#include <utils/delay.h>

//-----------------------------------------------------------------

#define PTR_U16(ptr, offset)         ((uint16_t*) &(ptr)[(offset)])
#define PTR_U32(ptr, offset)         ((uint32_t*) &(ptr)[(offset)])
#define PTR_FLOAT(ptr, offset)       ((float*) &(ptr)[(offset)])

#define DATA_U16(ptr, offset)        PTR_U16((ptr), MSG_POS_DATA + (offset))
#define DATA_U32(ptr, offset)        PTR_U32((ptr), MSG_POS_DATA + (offset))
#define DATA_FLOAT(ptr, offset)      PTR_FLOAT((ptr), MSG_POS_DATA + (offset))

//-----------------------------------------------------------------

const unsigned char CADDR_RX[] = RADIO_LOCAL_ADDR;
const unsigned char CADDR_TX[] = RADIO_REMOTE_ADDR;

//-----------------------------------------------------------------

static unsigned char dataBuffer[PAYLOAD_SIZE];

//-----------------------------------------------------------------

void commConfig(void)
{
    // nRF24 radio init
//    nrf24_config();
//    nrf24_initMinimal(true, PAYLOAD_SIZE, true);
//    nrf24_setRFChannel(23);
//    nrf24_setRxAddr((unsigned char*)ADDR_TX, 5, 0);
//    nrf24_setTxAddr((unsigned char*)ADDR_RX, 5);

    nrfInit();
    nrfSetRFChannel(23);
    nrfSetPayloadWidth(PAYLOAD_SIZE, 0);
    nrfAutoAckEnable(0);
    nrfSetTxAddr((unsigned char*)CADDR_TX, 5);
    nrfSetRxAddr((unsigned char*)CADDR_RX, 5, 0);
    nrfPowerUp();
    nrfSetAsTx();
    DELAY_MS(2);
    nrfClearFlush();
}

//-----------------------------------------------------------------

void commPollReceiver(void)
{
    nrfSetRxAddr((unsigned char*)CADDR_RX, 5, 0);

    nrfSetAsRx();

    while (!(nrfIsIRQActive() && NRF_CHECK_STATUS(NRF_IRQ_RX_DR)));

    nrfReadRxPayload(dataBuffer, PAYLOAD_SIZE);
    nrfClearAllIRQ();

    commProcessData();
}

//-----------------------------------------------------------------

bool commSendPayload(void)
{
    nrfSetTxAddr((unsigned char*)CADDR_TX, 5);
    nrfSetRxAddr((unsigned char*)CADDR_TX, 5, 0);

    DELAY_MS(1);

    nrfFlushTx();

    nrfSetAsTx();
    DELAY_MS(1);
    nrfWriteTxPayload(dataBuffer, PAYLOAD_SIZE, true);

    while (!(nrfIsIRQActive() && NRF_CHECK_STATUS(NRF_IRQ_TX_DS | NRF_IRQ_MAX_RT)));
    nrfClearAllIRQ();

    return true;
}

//-----------------------------------------------------------------

//-----------------------------------------------------------------

void commResponseOK(void)
{
    commCreateHeader(MSG_CMD_OK, MSG_CMD_OK, 0);
}

void commProcessData(void)
{
    commResponseOK();
    commSendPayload();
}


//-----------------------------------------------------------------

void commCreateHeader(uint8_t cmd, uint8_t resp, uint8_t len)
{
    dataBuffer[MSG_POS_START]    = MSG_START;
    dataBuffer[MSG_POS_LENGTH]   = len + 2;
    dataBuffer[MSG_POS_CMD]      = cmd;
    dataBuffer[MSG_POS_RESPONSE] = resp;
}


