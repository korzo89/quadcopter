/*
 * comm_cmd.h
 *
 *  Created on: 17-11-2013
 *      Author: Korzo
 */

#ifndef COMM_CMD_H_
#define COMM_CMD_H_

//-----------------------------------------------------------------

#include <stdbool.h>

//-----------------------------------------------------------------

//#define MSG_CMD_OK              0x00
//#define MSG_CMD_CONTROL         0x01
//#define MSG_CMD_ARM             0x02
//#define MSG_CMD_DISARM          0x03
//#define MSG_CMD_RAW_IMU         0x04
//#define MSG_CMD_ANGLES          0x05
//#define MSG_CMD_SET_PID_PITCH   0x06
//#define MSG_CMD_SET_PID_ROLL    0x07
//#define MSG_CMD_SET_PID_YAW     0x08

typedef enum
{
    MSG_CMD_OK = 0x00,
    MSG_CMD_CONTROL,
    MSG_CMD_ARM,
    MSG_CMD_DISARM,
    MSG_CMD_RAW_IMU,
    MSG_CMD_ANGLES,
    MSG_CMD_SET_PID_PITCH,
    MSG_CMD_SET_PID_ROLL,
    MSG_CMD_SET_PID_YAW,

    MSG_CMD_COUNT
} QuadCommand;

//-----------------------------------------------------------------

void commHandleMessage();

void cmdData(void);
void cmdControl(void);
void cmdArm(void);
void cmdDisarm(void);
void cmdSetPIDPitch(void);
void cmdSetPIDRoll(void);
void cmdSetPIDYaw(void);

void respOK(void);
void respRawIMU(void);
void respAngles(void);
void respPID(void);

void cmdExtractPID(bool *en, float *kp, float *ki, float *kd, float *mo, float *mi);

//-----------------------------------------------------------------

#endif /* COMM_CMD_H_ */
