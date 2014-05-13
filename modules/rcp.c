/*
 * rcp.c
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#include "rcp.h"
#include "comm_buffer.h"
#include "comm_cmd.h"

#include <drivers/nrf24l01.h>
#include <drivers/led.h>
#include <drivers/motors.h>
#include <utils/delay.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <portmacro.h>

//-----------------------------------------------------------------

#define RCP_RF_CHANNEL          23
#define RCP_PIPE                0

#define RCP_LOCAL_ADDR          "quad0"
#define RCP_REMOTE_ADDR         "ctrl0"
#define RCP_ADDR_LEN            5

#define RCP_TX_QUEUE_SIZE       3

//-----------------------------------------------------------------

const unsigned char ADDR_RX[] = RCP_LOCAL_ADDR;
const unsigned char ADDR_TX[] = RCP_REMOTE_ADDR;

//-----------------------------------------------------------------

CommBuffer buffer;

volatile int lostCount = 0;

static xSemaphoreHandle rcpReady;
static xQueueHandle txQueue;

static RCPCallback cmdCallbacks[RCP_CMD_NUM] = { 0 };
static RCPCallback queryCallbacks[RCP_CMD_NUM] = { 0 };

//-----------------------------------------------------------------

static void rcpRadioIRQCallback(void)
{
    portBASE_TYPE woken = pdFALSE;
    xSemaphoreGiveFromISR(rcpReady, &woken);

    portEND_SWITCHING_ISR(woken);
}

//-----------------------------------------------------------------

void rcpInit(void)
{
    vSemaphoreCreateBinary(rcpReady);
    txQueue = xQueueCreate(RCP_TX_QUEUE_SIZE, sizeof(RCPMessage));

    // nRF24 radio init
    nrfInit();
    nrfSetRFChannel(RCP_RF_CHANNEL);
    nrfSetPayloadWidth(RCP_PAYLOAD_SIZE, RCP_PIPE);
    nrfAutoAckEnable(RCP_PIPE);
    nrfSetIRQCallback(rcpRadioIRQCallback);

    DELAY_MS(2);
    nrfPowerUp();
    nrfClearFlush();
}

//-----------------------------------------------------------------

void rcpTask(void *args)
{
    RCPMessage msg;

    while (1)
    {
        // wait for IRQ
        xSemaphoreTake(rcpReady, portMAX_DELAY);

        // get all available RX payload data
        rcpEnableRx();
        while (!(nrfGetFIFOStatus() & NRF_FIFO_STATUS_RX_EMPTY))
        {
            nrfReadRxPayload(msg.raw, RCP_PAYLOAD_SIZE);
            rcpProcessMessage(&msg);
        }
        nrfClearIRQ(NRF_IRQ_RX_DR);

        // send all pending messages
        rcpEnableTx();
        while (uxQueueMessagesWaiting(txQueue) > 0)
        {
            xQueueReceive(txQueue, &msg, 0);

            nrfWriteTxPayload(msg.raw, RCP_PAYLOAD_SIZE, true);

            // TODO: wait for TX interrupt
        }
    }
}

//-----------------------------------------------------------------

bool rcpSendMessage(RCPMessage *msg)
{
    return xQueueSend(txQueue, msg, portMAX_DELAY) == pdTRUE;
}

//-----------------------------------------------------------------

void rcpRegisterCallback(RCPCommand cmd, RCPCallback callback, bool query)
{
    if (query)
        queryCallbacks[(int)cmd] = callback;
    else
        cmdCallbacks[(int)cmd] = callback;
}

//-----------------------------------------------------------------

void rcpEnableRx(void)
{
    nrfSetRxAddr((unsigned char*)ADDR_RX, RCP_ADDR_LEN, RCP_PIPE);
    nrfSetAsRx();
    DELAY_MS(1);
}

//-----------------------------------------------------------------

void rcpEnableTx(void)
{
    nrfSetTxAddr((unsigned char*)ADDR_TX, RCP_ADDR_LEN);
    nrfSetRxAddr((unsigned char*)ADDR_TX, RCP_ADDR_LEN, 0);
    nrfSetAsTx();
}

//-----------------------------------------------------------------

void rcpProcessMessage(RCPMessage *msg)
{
    if (!msg || !IS_VALID_CMD(msg->cmd))
        return;

    RCPCallback callback = cmdCallbacks[(int)msg->cmd];
    if (callback)
        callback(msg);

    callback = queryCallbacks[(int)msg->query];
    if (callback)
        callback(msg);
}

//-----------------------------------------------------------------

//void rcpPollReceiver(void)
//{
//    nrfSetRxAddr((unsigned char*)ADDR_RX, RCP_ADDR_LEN, RCP_PIPE);
//    nrfSetAsRx();
//
//    if (xSemaphoreTake)
//    while (!(nrf24_irqPinActive() && nrf24_irq_RX_DR()));
//
//    nrf24_readRxPayload(buffer.data, PAYLOAD_SIZE);
//    nrf24_irqClearAll();
//
//    rcpProcessData();
//}
//
//bool rcpSendPayload(void)
//{
//    nrf24_setTxAddr((unsigned char*) ADDR_TX, 5);
//    nrf24_setRxAddr((unsigned char*) ADDR_TX, 5, 0);
//
//    nrf24_delay(1);
//
//    nrf24_flushTx();
//
//    nrf24_setAsTx();
//    nrf24_delay(1);
//    nrf24_writeTxPayload(buffer.data, PAYLOAD_SIZE, true);
//
//    while (!(nrf24_irqPinActive() && (nrf24_irq_TX_DS() || nrf24_irq_MAX_RT())));
//    nrf24_irqClearAll();
//
//    return true;
//}
//
//void rcpProcessData(void)
//{
//    commBufferReadHeader(&buffer);
//    if (buffer.start != MSG_START)
//        return;
//
//    commHandleMessage();
//
//    ledTurnOn(LED_RED);
//    commSendPayload();
//    ledTurnOff(LED_RED);
//
//    lostCount = 0;
//}
