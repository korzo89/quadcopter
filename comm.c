/*
 * comm.c
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#include "comm.h"
#include <drivers/nrf24l01.h>
#include <utils/delay.h>
#include <drivers/led.h>

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

    nrf_init();
    nrf_set_rf_channel(23);
    nrf_set_payload_width(PAYLOAD_SIZE, 0);
    nrf_auto_ack_enable(0);
    nrf_set_tx_addr((unsigned char*)CADDR_TX, 5);
    nrf_set_rx_addr((unsigned char*)CADDR_RX, 5, 0);
    nrf_power_up();
    nrf_set_as_tx();
    DELAY_MS(2);
    nrf_clear_flush();
}

//-----------------------------------------------------------------

void commPollReceiver(void)
{
    nrf_set_rx_addr((unsigned char*)CADDR_RX, 5, 0);

    nrf_set_as_rx();

    while (!(nrf_is_irq_active() && NRF_CHECK_STATUS(NRF_IRQ_RX_DR)));

    nrf_read_rx_payload(dataBuffer, PAYLOAD_SIZE);
    nrf_clear_all_irq();

    commProcessData();
}

//-----------------------------------------------------------------

bool commSendPayload(void)
{
    nrf_set_tx_addr((unsigned char*)CADDR_TX, 5);
    nrf_set_rx_addr((unsigned char*)CADDR_TX, 5, 0);

    DELAY_MS(1);

    nrf_flush_tx();

    nrf_set_as_tx();
    DELAY_MS(1);
    nrf_write_tx_payload(dataBuffer, PAYLOAD_SIZE, true);

    while (!(nrf_is_irq_active() && NRF_CHECK_STATUS(NRF_IRQ_TX_DS | NRF_IRQ_MAX_RT)));
    nrf_clear_all_irq();

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
	led_turn_on(LED_RED);
    commResponseOK();
    commSendPayload();
    led_turn_off(LED_RED);
}


//-----------------------------------------------------------------

void commCreateHeader(uint8_t cmd, uint8_t resp, uint8_t len)
{
    dataBuffer[MSG_POS_START]    = MSG_START;
    dataBuffer[MSG_POS_LENGTH]   = len + 2;
    dataBuffer[MSG_POS_CMD]      = cmd;
    dataBuffer[MSG_POS_RESPONSE] = resp;
}


