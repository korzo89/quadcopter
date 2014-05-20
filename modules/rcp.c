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

#define RCP_RX_MAX_DELAY		500
#define RCP_TX_MAX_DELAY		100

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

static void rcpTask(void *args)
{
    RCPMessage msg;
    portBASE_TYPE res;

//    xSemaphoreTake(rcpReady, 0);

    while (1)
    {
    	// switch to RX mode
    	xSemaphoreTake(rcpReady, 0);
        rcpEnableRx();
        // wait for ready interrupt
        res = xSemaphoreTake(rcpReady, MSEC_TO_TICKS(RCP_RX_MAX_DELAY));
        // check for RX interrupt or timeout
		if (res == pdTRUE && NRF_CHECK_STATUS(NRF_IRQ_RX_DR))
        {
			nrfClearIRQ(NRF_IRQ_RX_DR);

			// read all pending messages
        	while (!NRF_CHECK_FIFO(NRF_FIFO_STATUS_RX_EMPTY))
        	{
        		ledTurnOn(LED_RED);

				nrfReadRxPayload(msg.raw, RCP_PAYLOAD_SIZE);
	        	rcpProcessMessage(&msg);

	        	ledTurnOff(LED_RED);
        	}
        }

		// send all pending messages
		rcpEnableTx();
		while (uxQueueMessagesWaiting(txQueue) > 0)
		{
			xQueueReceive(txQueue, &msg, 0);

			ledTurnOn(LED_YELLOW);

			xSemaphoreTake(rcpReady, 0);
			nrfWriteTxPayload(msg.raw, RCP_PAYLOAD_SIZE, true);

			// wait for interrupt
			res = xSemaphoreTake(rcpReady, MSEC_TO_TICKS(RCP_TX_MAX_DELAY));
			if (res == pdTRUE)
			{
				if (NRF_CHECK_STATUS(NRF_IRQ_TX_DS | NRF_IRQ_MAX_RT))
					nrfClearIRQ(NRF_IRQ_TX_DS | NRF_IRQ_MAX_RT);
			}

			ledTurnOff(LED_YELLOW);
		}

		nrfClearAllIRQ();
    }
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

    nrfSetTxAddr((unsigned char*)ADDR_TX, RCP_ADDR_LEN);
    nrfSetRxAddr((unsigned char*)ADDR_RX, RCP_ADDR_LEN, RCP_PIPE);

    DELAY_MS(2);
    nrfPowerUp();
    nrfClearFlush();

    xTaskCreate(rcpTask, (signed portCHAR*)"RCP",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);
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

    if (IS_VALID_CMD(msg->query))
    {
		callback = queryCallbacks[(int)msg->query];
		if (callback)
			callback(msg);
    }
}
