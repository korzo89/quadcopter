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

static void rcpTask(void *args)
{
    RCPMessage msg;

//    xSemaphoreTake(rcpReady, 0);

    while (1)
    {
//        rcpEnableRx();
//
//        char addr[6];
//        nrfReadRegister(NRF_RX_ADDR_P0, addr, 5);
//        addr[5] = '\0';
//        UARTprintf("%s\n", addr);
//
//        while (!nrfIsIRQActive());
//        uint8_t status = nrfGetStatus();
//
//        // wait for IRQ
//        xSemaphoreTake(rcpReady, portMAX_DELAY);
//
//        // get all available RX payload data
//        while (!(nrfGetFIFOStatus() & NRF_FIFO_STATUS_RX_EMPTY))
//        {
//            nrfReadRxPayload(msg.raw, RCP_PAYLOAD_SIZE);
//            rcpProcessMessage(&msg);
//        }
//        nrfClearIRQ(NRF_IRQ_RX_DR);
//
//        // send all pending messages
//        rcpEnableTx();
//        while (uxQueueMessagesWaiting(txQueue) > 0)
//        {
//            xQueueReceive(txQueue, &msg, 0);
//
//            nrfWriteTxPayload(msg.raw, RCP_PAYLOAD_SIZE, true);
//
//            // TODO: wait for TX interrupt
//        }

    	ledTurnOn(LED_RED);

        rcpEnableRx();
        // wait for ready interrupt
        xSemaphoreTake(rcpReady, 0);
        xSemaphoreTake(rcpReady, portMAX_DELAY);
        if (NRF_CHECK_STATUS(NRF_IRQ_RX_DR))
        {
        	nrfReadRxPayload(msg.raw, RCP_PAYLOAD_SIZE);
//        	rcpProcessMessage(&msg);

			nrfClearIRQ(NRF_IRQ_RX_DR);

			static int cc = 0;
			memset(msg.data, cc++, 10);
			msg.cmd = 0x05;
			msg.query = 0x00;
			rcpSendMessage(&msg);

//			static bool on = false;
//			ledToggle(LED_RED, on);
//			on = !on;
        }

		// send all pending messages
		rcpEnableTx();
		while (uxQueueMessagesWaiting(txQueue) > 0)
		{
			xQueueReceive(txQueue, &msg, 0);

			xSemaphoreTake(rcpReady, 0);
			nrfWriteTxPayload(msg.raw, RCP_PAYLOAD_SIZE, true);

			xSemaphoreTake(rcpReady, portMAX_DELAY);
			if (NRF_CHECK_STATUS(NRF_IRQ_TX_DS | NRF_IRQ_MAX_RT))
				nrfClearIRQ(NRF_IRQ_TX_DS | NRF_IRQ_MAX_RT);
		}

		ledTurnOff(LED_RED);
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

    uint8_t reg, temp;
    for (reg = 0x00; reg <= 0x17; ++reg)
    {
        temp = nrfReadRegisterByte(reg);
        UARTprintf("reg 0x%02x: 0x%02x (%d)\r\n", reg, temp, temp);
    }

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

    callback = queryCallbacks[(int)msg->query];
    if (callback)
        callback(msg);
}
