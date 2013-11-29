/*
 * comm.c
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#include "comm.h"
#include "comm_buffer.h"
#include "nrf24l01/nrf24l01.h"
#include "imu/imu.h"
#include "pid.h"
#include "control.h"
#include "motors.h"
#include "led.h"

//-----------------------------------------------------------------

extern int16_t accX, accY, accZ;       // raw accelerometer values      6
extern int16_t gyroX, gyroY, gyroZ;    // raw gyroscope values          6
extern int16_t magX, magY, magZ;       // raw magnetometer values       6
extern int16_t temperature;            // raw temperature               2
extern int32_t pressure;               // raw pressure                  4 = 24

//-----------------------------------------------------------------

const unsigned char ADDR_RX[] = RADIO_LOCAL_ADDR;
const unsigned char ADDR_TX[] = RADIO_REMOTE_ADDR;

//-----------------------------------------------------------------

static CommBuffer buffer;

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

    // handle incoming command
    switch (buffer.command)
    {
    case MSG_CMD_CONTROL:
        commProcessControl();
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
    switch (buffer.response)
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

    default:
        break;
    }

    LEDTurnOn(LED_RED);

    commSendPayload();

    LEDTurnOff(LED_RED);

    lostCount = 0;
}

//-----------------------------------------------------------------

void commProcessControl(void)
{
    uint16_t m1, m2, m3, m4;

    if (!motorsArmed())
        return;

    commBufferResetRead(&buffer);
    COMM_BUFFER_READ_T(&buffer, &m1, uint16_t);
    COMM_BUFFER_READ_T(&buffer, &m2, uint16_t);
    COMM_BUFFER_READ_T(&buffer, &m3, uint16_t);
    COMM_BUFFER_READ_T(&buffer, &m4, uint16_t);

    motorsSetThrottle((float)m1, (float)m2, (float)m3, (float)m4);
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

void commSetPIDPitch(void)
{
    bool en;
    float kp, ki, kd, mo, mi;

    commExtractPID(&en, &kp, &ki, &kd, &mo, &mi);
    controlSetPitchPID(en, kp, ki, kd, mo, 0.0f, mi);
}

void commSetPIDRoll(void)
{
    bool en;
    float kp, ki, kd, mo, mi;

    commExtractPID(&en, &kp, &ki, &kd, &mo, &mi);
    controlSetRollPID(en, kp, ki, kd, mo, 0.0f, mi);
}

//-----------------------------------------------------------------

void commSetPIDYaw(void)
{
    bool en;
    float kp, ki, kd, mo, mi;
    commExtractPID(&en, &kp, &ki, &kd, &mo, &mi);
    controlSetYawPID(en, kp, ki, kd, mo, 0.0f, mi);
}

//-----------------------------------------------------------------

void commExtractPID(bool *en, float *kp, float *ki, float *kd, float *mo, float *mi)
{
    commBufferResetRead(&buffer);
    COMM_BUFFER_READ_T(&buffer, en, bool);
    COMM_BUFFER_READ_T(&buffer, kp, float);
    COMM_BUFFER_READ_T(&buffer, ki, float);
    COMM_BUFFER_READ_T(&buffer, kd, float);
    COMM_BUFFER_READ_T(&buffer, mo, float);
    COMM_BUFFER_READ_T(&buffer, mi, float);
}

//-----------------------------------------------------------------

void commResponseOK(void)
{
    commBufferCreateHeader(&buffer, MSG_CMD_OK, MSG_CMD_OK);
}

//-----------------------------------------------------------------

void commResponseRawIMU(void)
{
    commBufferResetWrite(&buffer);
    COMM_BUFFER_WRITE_T(&buffer, &accX, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &accY, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &accZ, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &gyroX, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &gyroY, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &gyroZ, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &magX, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &magY, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &magZ, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &pressure, uint32_t);
    COMM_BUFFER_WRITE_T(&buffer, &temperature, uint16_t);
    commBufferCreateHeader(&buffer, MSG_CMD_RAW_IMU, MSG_CMD_OK);
}

//-----------------------------------------------------------------

void commResponseAngles(void)
{
    float pitch, roll, yaw;

    IMUGetEulerAngles(&pitch, &roll, &yaw);

    commBufferResetWrite(&buffer);
    COMM_BUFFER_WRITE_T(&buffer, &pitch, float);
    COMM_BUFFER_WRITE_T(&buffer, &roll, float);
    COMM_BUFFER_WRITE_T(&buffer, &yaw, float);
    commBufferCreateHeader(&buffer, MSG_CMD_ANGLES, MSG_CMD_OK);
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
