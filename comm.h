/*
 * comm.h
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#ifndef COMM_H_
#define COMM_H_

//-----------------------------------------------------------------

#include <stdbool.h>
#include <stdint.h>

//-----------------------------------------------------------------

#define PAYLOAD_SIZE            32

#define RADIO_LOCAL_ADDR        "quad0"
#define RADIO_REMOTE_ADDR       "ctrl0"

#define MSG_POS_START           0
#define MSG_POS_LENGTH          1
#define MSG_POS_CMD             2
#define MSG_POS_RESPONSE        3
#define MSG_POS_DATA            4

#define MSG_START               0x99
#define MSG_CMD_OK              0x00
#define MSG_CMD_CONTROL         0x01
#define MSG_CMD_ARM             0x02
#define MSG_CMD_DISARM          0x03
#define MSG_CMD_RAW_IMU         0x04
#define MSG_CMD_ANGLES          0x05
#define MSG_CMD_SET_PITCH_PID   0x06
#define MSG_CMD_SET_ROLL_PID    0x07
#define MSG_CMD_SET_YAW_PID     0x08

//-----------------------------------------------------------------

void commConfig(void);

void commPollReceiver(void);
bool commSendPayload(void);

void commCreateHeader(uint8_t cmd, uint8_t resp, uint8_t len);

void commProcessData(void);

//-----------------------------------------------------------------

#endif /* COMM_H_ */
