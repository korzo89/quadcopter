/*
 * rcp.c
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#include "rcp.h"

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

#define RCP_RX_MAX_DELAY		100
#define RCP_TX_MAX_DELAY		100

#define RCP_DISCONNECT_TIME     1500

//-----------------------------------------------------------------

static const unsigned char ADDR_RX[] = RCP_LOCAL_ADDR;
static const unsigned char ADDR_TX[] = RCP_REMOTE_ADDR;

//-----------------------------------------------------------------

static xSemaphoreHandle rcp_ready;
static xQueueHandle tx_queue;

static rcp_callback_t cmd_callbacks[RCP_CMD_NUM]    = { 0 };
static rcp_callback_t query_callbacks[RCP_CMD_NUM]  = { 0 };

static portTickType last_msg_time = 0;

//-----------------------------------------------------------------

static void rcp_radio_irq_callback(void)
{
    portBASE_TYPE woken = pdFALSE;
    xSemaphoreGiveFromISR(rcp_ready, &woken);

    portEND_SWITCHING_ISR(woken);
}

//-----------------------------------------------------------------

static void rcp_task(void *args)
{
    rcp_message_t msg;
    portBASE_TYPE res;

    while (1)
    {
        // switch to RX mode
        xSemaphoreTake(rcp_ready, 0);
        rcp_enable_rx();
        // wait for ready interrupt
        res = xSemaphoreTake(rcp_ready, MSEC_TO_TICKS(RCP_RX_MAX_DELAY));
        // check for RX interrupt or timeout
        if (res == pdTRUE && NRF_CHECK_STATUS(NRF_IRQ_RX_DR))
        {
            nrf_clear_irq(NRF_IRQ_RX_DR);

            last_msg_time = xTaskGetTickCount();

            // read all pending messages
            while (!NRF_CHECK_FIFO(NRF_FIFO_STATUS_RX_EMPTY))
            {
                led_turn_on(LED_RED);

                nrf_read_rx_payload(msg.raw, RCP_PAYLOAD_SIZE);
                rcp_process_message(&msg);

                led_turn_off(LED_RED);
            }
        }

        // send all pending messages
        nrf_flush_tx();
        rcp_enable_tx();
        while (uxQueueMessagesWaiting(tx_queue) > 0)
        {
            xQueueReceive(tx_queue, &msg, 0);

            led_turn_on(LED_YELLOW);

            xSemaphoreTake(rcp_ready, 0);
            nrf_write_tx_payload(msg.raw, RCP_PAYLOAD_SIZE, true);

            // wait for interrupt
            res = xSemaphoreTake(rcp_ready, MSEC_TO_TICKS(RCP_TX_MAX_DELAY));
            if (res == pdTRUE)
            {
                if (NRF_CHECK_STATUS(NRF_IRQ_TX_DS | NRF_IRQ_MAX_RT))
                    nrf_clear_irq(NRF_IRQ_TX_DS | NRF_IRQ_MAX_RT);
            }

            led_turn_off(LED_YELLOW);
        }

        nrf_clear_all_irq();
    }
}

//-----------------------------------------------------------------

void rcp_init(void)
{
    vSemaphoreCreateBinary(rcp_ready);
    tx_queue = xQueueCreate(RCP_TX_QUEUE_SIZE, sizeof(rcp_message_t));

    // nRF24 radio init
    nrf_init();
    nrf_set_rf_channel(RCP_RF_CHANNEL);
    nrf_set_payload_width(RCP_PAYLOAD_SIZE, RCP_PIPE);
    nrf_auto_ack_enable(RCP_PIPE);
    nrf_set_irq_callback(rcp_radio_irq_callback);

    nrf_set_tx_addr((unsigned char*)ADDR_TX, RCP_ADDR_LEN);
    nrf_set_rx_addr((unsigned char*)ADDR_RX, RCP_ADDR_LEN, RCP_PIPE);

    DELAY_MS(2);
    nrf_power_up();
    nrf_clear_flush();

    xTaskCreate(rcp_task, (signed portCHAR*)"RCP",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

//-----------------------------------------------------------------

bool rcp_send_message(rcp_message_t *msg)
{
    return xQueueSend(tx_queue, msg, portMAX_DELAY) == pdTRUE;
}

//-----------------------------------------------------------------

void rcp_register_callback(rcp_command_t cmd, rcp_callback_t callback, bool query)
{
    if (query)
        query_callbacks[(int)cmd] = callback;
    else
        cmd_callbacks[(int)cmd] = callback;
}

//-----------------------------------------------------------------

void rcp_enable_rx(void)
{
    nrf_set_rx_addr((unsigned char*)ADDR_RX, RCP_ADDR_LEN, RCP_PIPE);
    nrf_set_as_rx();
    DELAY_MS(1);
}

//-----------------------------------------------------------------

void rcp_enable_tx(void)
{
    nrf_set_tx_addr((unsigned char*)ADDR_TX, RCP_ADDR_LEN);
    nrf_set_rx_addr((unsigned char*)ADDR_TX, RCP_ADDR_LEN, 0);
    nrf_set_as_tx();
}

//-----------------------------------------------------------------

void rcp_process_message(rcp_message_t *msg)
{
    if (!msg || !IS_VALID_CMD(msg->packet.cmd))
        return;

    rcp_callback_t callback = cmd_callbacks[(int)msg->packet.cmd];
    if (callback)
        callback(msg);

    if (IS_VALID_CMD(msg->packet.query))
    {
        callback = query_callbacks[(int)msg->packet.query];
        if (callback)
            callback(msg);
    }
}

//-----------------------------------------------------------------

bool rcp_is_connected(void)
{
    return last_msg_time &&
            (xTaskGetTickCount() - last_msg_time <= MSEC_TO_TICKS(RCP_DISCONNECT_TIME));
}
