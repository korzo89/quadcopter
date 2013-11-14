/*
 * comm.c
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#include "comm.h"
#include "nrf24l01/nrf24l01.h"
#include "imu/imu.h"
#include "pid.h"
#include "motors.h"
#include "led.h"

//-----------------------------------------------------------------

#define PTR_U16(ptr, offset)         ((uint16_t*) &(ptr)[(offset)])
#define PTR_U32(ptr, offset)         ((uint32_t*) &(ptr)[(offset)])
#define PTR_FLOAT(ptr, offset)       ((float*) &(ptr)[(offset)])

#define DATA_U16(ptr, offset)        PTR_U16((ptr), MSG_POS_DATA + (offset))
#define DATA_U32(ptr, offset)        PTR_U32((ptr), MSG_POS_DATA + (offset))
#define DATA_FLOAT(ptr, offset)      PTR_FLOAT((ptr), MSG_POS_DATA + (offset))

//-----------------------------------------------------------------

extern int16_t accX, accY, accZ;       // raw accelerometer values      6
extern int16_t gyroX, gyroY, gyroZ;    // raw gyroscope values          6
extern int16_t magX, magY, magZ;       // raw magnetometer values       6
extern int16_t temperature;            // raw temperature               2
extern int32_t pressure;               // raw pressure                  4 = 24

extern PID_t pitchPID;

//-----------------------------------------------------------------

const unsigned char ADDR_RX[] = RADIO_LOCAL_ADDR;
const unsigned char ADDR_TX[] = RADIO_REMOTE_ADDR;

//-----------------------------------------------------------------

static unsigned char dataBuffer[PAYLOAD_SIZE];

volatile int lostCount = 0;

//-----------------------------------------------------------------

void commConfig(void)
{
    // nRF24 radio init
    nrf24_config();
    nrf24_initMinimal(true, PAYLOAD_SIZE, true);
    nrf24_setRFChannel(23);
    nrf24_setRxAddr((unsigned char*)ADDR_TX, 5, 0);
    nrf24_setTxAddr((unsigned char*)ADDR_RX, 5);
}

//-----------------------------------------------------------------

void commPollReceiver(void)
{
    nrf24_setRxAddr((unsigned char*)ADDR_RX, 5, 0);

    nrf24_setAsRx(true);

    while (!(nrf24_irqPinActive() && nrf24_irq_RX_DR()));

    nrf24_readRxPayload(dataBuffer, PAYLOAD_SIZE);
    nrf24_irqClearAll();

    commProcessData();
}

//-----------------------------------------------------------------

bool commSendPayload(void)
{
    nrf24_setTxAddr((unsigned char*)ADDR_TX, 5);
    nrf24_setRxAddr((unsigned char*)ADDR_TX, 5, 0);

    nrf24_delay(1);

    nrf24_flushTx();

    nrf24_setAsTx();
    nrf24_delay(1);
    nrf24_writeTxPayload(dataBuffer, PAYLOAD_SIZE, true);

    while (!(nrf24_irqPinActive() && (nrf24_irq_TX_DS() || nrf24_irq_MAX_RT())));
    nrf24_irqClearAll();

    return true;
}

//-----------------------------------------------------------------

void commProcessData(void)
{
    if (dataBuffer[MSG_POS_START] != MSG_START)
        return;

    // handle incoming command
    switch (dataBuffer[MSG_POS_CMD])
    {
    case MSG_CMD_SET_THROTTLE:
        commProcessSetThrottle();
        break;

    case MSG_CMD_ARM:
        commProcessArm();
        break;

    case MSG_CMD_DISARM:
        commProcessDisarm();
        break;

    default:
        break;
    }

    // handle response request
    switch (dataBuffer[MSG_POS_RESPONSE])
    {
    case MSG_CMD_OK:
        commResponseOK();
        break;

    case MSG_CMD_RAW_IMU:
        commResponseRawIMU();
        break;

    case MSG_CMD_ANGLES:
        commResponseAngles();
        break;

    case MSG_CMD_PID:
        commResponsePID();
        break;

    default:
        break;
    }

    LEDTurnOn(LED_RED);

    commSendPayload();

    LEDTurnOff(LED_RED);

    lostCount = 0;
}

//-----------------------------------------------------------------

void commProcessSetThrottle(void)
{
    float m1, m2, m3, m4;

    if (!motorsArmed())
        return;

    m1 = (float) *((uint16_t*) &dataBuffer[MSG_POS_DATA]);
    m2 = (float) *((uint16_t*) &dataBuffer[MSG_POS_DATA + 2]);
    m3 = (float) *((uint16_t*) &dataBuffer[MSG_POS_DATA + 4]);
    m4 = (float) *((uint16_t*) &dataBuffer[MSG_POS_DATA + 6]);

    motorsSetThrottle(m1, m2, m3, m4);
}

//-----------------------------------------------------------------

void commProcessArm(void)
{
    motorsArm();
}

//-----------------------------------------------------------------

void commProcessDisarm(void)
{
    motorsDisarm();
}

//-----------------------------------------------------------------

void commCreateHeader(uint8_t cmd, uint8_t resp, uint8_t len)
{
    dataBuffer[MSG_POS_START]    = MSG_START;
    dataBuffer[MSG_POS_LENGTH]   = len + 2;
    dataBuffer[MSG_POS_CMD]      = cmd;
    dataBuffer[MSG_POS_RESPONSE] = resp;
}

//-----------------------------------------------------------------

void commResponseOK(void)
{
    commCreateHeader(MSG_CMD_OK, MSG_CMD_OK, 0);
}

//-----------------------------------------------------------------

void commResponseRawIMU(void)
{
    commCreateHeader(MSG_CMD_RAW_IMU, MSG_CMD_OK, 24);
    *DATA_U16(dataBuffer, 0)     = accX;
    *DATA_U16(dataBuffer, 2)     = accY;
    *DATA_U16(dataBuffer, 4)     = accZ;
    *DATA_U16(dataBuffer, 6)     = gyroX;
    *DATA_U16(dataBuffer, 8)     = gyroY;
    *DATA_U16(dataBuffer, 10)    = gyroZ;
    *DATA_U16(dataBuffer, 12)    = magX;
    *DATA_U16(dataBuffer, 14)    = magY;
    *DATA_U16(dataBuffer, 16)    = magZ;
    *DATA_U32(dataBuffer, 18)    = pressure;
    *DATA_U16(dataBuffer, 22)    = temperature;
}

//-----------------------------------------------------------------

void commResponseAngles(void)
{
    float pitch, roll, yaw;

    IMUGetEulerAngles(&pitch, &roll, &yaw);

    commCreateHeader(MSG_CMD_ANGLES, MSG_CMD_OK, 12);
    *DATA_FLOAT(dataBuffer, 0)   = pitch;
    *DATA_FLOAT(dataBuffer, 4)   = roll;
    *DATA_FLOAT(dataBuffer, 8)   = yaw;
}

//-----------------------------------------------------------------

void commResponsePID(void)
{
    commCreateHeader(MSG_CMD_PID, MSG_CMD_OK, 4);
    *DATA_FLOAT(dataBuffer, 0) = pitchPID.output;
}

//-----------------------------------------------------------------

void Timer3AIntHandler(void)
{
//    if (!TimerIntStatus(TIMER3_BASE, TIMER_TIMA_TIMEOUT))
//        return;

    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    lostCount++;
    if (motorsArmed() && lostCount == 5)
        motorsDisarm();
}
