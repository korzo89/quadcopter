/*
 * comm.c
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#include "comm.h"
#include "nrf24/nrf24.h"
#include "servo/servo.h"

#include <stdlib.h>
#include "utils/uartstdio.h"

//-----------------------------------------------------------------

const unsigned char ADDR_RX[] = RADIO_LOCAL_ADDR;
const unsigned char ADDR_TX[] = RADIO_REMOTE_ADDR;

//-----------------------------------------------------------------

static unsigned char dataBuffer[PAYLOAD_SIZE];

//-----------------------------------------------------------------

void commConfig(void)
{
    // nRF24 radio init
    nRF24_config();
    nRF24_init();
    nRF24_setChannel(23);
    nRF24_setPayloadSize(PAYLOAD_SIZE);
    nRF24_setRF(NRF24DataRate2Mbps, NRF24TransmitPower0dBm);
}

//-----------------------------------------------------------------

void commPollReceiver(void)
{
    uint8_t len;

//    UARTprintf("Receiving...\r\n");

    nRF24_setRxAddress((uint8_t*) ADDR_RX);
//    nRF24_waitAvailable();
    if (!nRF24_waitAvailableTimeout(10))
        return;

    nRF24_receive(dataBuffer, &len);

    commProcessData();

//    UARTprintf("Received: %s\r\n", dataBuffer);

    nRF24_setTxAddress((uint8_t*) ADDR_TX);
    nRF24_send(dataBuffer, PAYLOAD_SIZE, false);
    if (!nRF24_waitPacketSent())
        return;
}

//-----------------------------------------------------------------

void commProcessData(void)
{
    uint16_t mot1, mot2, mot3, mot4;

    UARTprintf("Received: %x %d %x\r\n", dataBuffer[0], dataBuffer[1], dataBuffer[2]);
    if (dataBuffer[0] != MSG_START)
        return;

    switch (dataBuffer[2])
    {
    case MSG_SET_MOTORS:
        mot1 = 1000 + *((uint16_t*) &dataBuffer[3]);
        mot2 = 1000 + *((uint16_t*) &dataBuffer[5]);
        mot3 = 1000 + *((uint16_t*) &dataBuffer[7]);
        mot4 = 1000 + *((uint16_t*) &dataBuffer[9]);

        servoSetPulse(SERVO0_BASE, SERVO0_TIMER, mot1);
        servoSetPulse(SERVO1_BASE, SERVO1_TIMER, mot2);
        servoSetPulse(SERVO2_BASE, SERVO2_TIMER, mot3);
        servoSetPulse(SERVO3_BASE, SERVO3_TIMER, mot4);

//        UARTprintf("SET_MOTORS: %d, %d, %d, %d\r\n",
//                 *((uint16_t*) &dataBuffer[3]),
//                 *((uint16_t*) &dataBuffer[5]),
//                 *((uint16_t*) &dataBuffer[7]),
//                 *((uint16_t*) &dataBuffer[9]));
        break;

    default:
        break;
    }
}
