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
#include "comm_cmd.h"

//-----------------------------------------------------------------

#define PAYLOAD_SIZE            32

#define RADIO_LOCAL_ADDR        "quad0"
#define RADIO_REMOTE_ADDR       "ctrl0"

//-----------------------------------------------------------------

void commConfig(void);

void commPollReceiver(void);
bool commSendPayload(void);

void commProcessData(void);
void commProcessControl(void);
void commProcessArm(void);
void commProcessDisarm(void);
void commSetPIDPitch(void);
void commSetPIDRoll(void);
void commSetPIDYaw(void);

void commResponseOK(void);
void commResponseRawIMU(void);
void commResponseAngles(void);
void commResponsePID(void);

void commExtractPID(bool *en, float *kp, float *ki, float *kd, float *mo, float *mi);

//-----------------------------------------------------------------

#endif /* COMM_H_ */
