/*
 * comm.c
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#include "comm.h"
#include "comm_buffer.h"
#include "comm_cmd.h"
#include "nrf24l01/nrf24l01.h"
#include <drivers/led.h>
#include <drivers/motors.h>

#include <driverlib/timer.h>

//-----------------------------------------------------------------

const unsigned char ADDR_RX[] = RADIO_LOCAL_ADDR;
const unsigned char ADDR_TX[] = RADIO_REMOTE_ADDR;

//-----------------------------------------------------------------

CommBuffer buffer;

volatile int lostCount = 0;

//-----------------------------------------------------------------

void commConfig(void)
{
    // nRF24 radio init
    nrf24_config();
    nrf24_initMinimal(true, PAYLOAD_SIZE, true);
    nrf24_setRFChannel(23);
    nrf24_setRxAddr((unsigned char*) ADDR_TX, 5, 0);
    nrf24_setTxAddr((unsigned char*) ADDR_RX, 5);
}

//-----------------------------------------------------------------

void commPollReceiver(void)
{
    nrf24_setRxAddr((unsigned char*) ADDR_RX, 5, 0);

    nrf24_setAsRx(true);

    while (!(nrf24_irqPinActive() && nrf24_irq_RX_DR()));

    nrf24_readRxPayload(buffer.data, PAYLOAD_SIZE);
    nrf24_irqClearAll();

    commProcessData();
}

//-----------------------------------------------------------------

bool commSendPayload(void)
{
    nrf24_setTxAddr((unsigned char*) ADDR_TX, 5);
    nrf24_setRxAddr((unsigned char*) ADDR_TX, 5, 0);

    nrf24_delay(1);

    nrf24_flushTx();

    nrf24_setAsTx();
    nrf24_delay(1);
    nrf24_writeTxPayload(buffer.data, PAYLOAD_SIZE, true);

    while (!(nrf24_irqPinActive() && (nrf24_irq_TX_DS() || nrf24_irq_MAX_RT())));
    nrf24_irqClearAll();

    return true;
}

//-----------------------------------------------------------------

void commProcessData(void)
{
    commBufferReadHeader(&buffer);
    if (buffer.start != MSG_START)
        return;

    commHandleMessage();

    ledTurnOn(LED_RED);
    commSendPayload();
    ledTurnOff(LED_RED);

    lostCount = 0;
}
