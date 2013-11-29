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

//-----------------------------------------------------------------

#define PAYLOAD_SIZE            32

#define RADIO_LOCAL_ADDR        "quad0"
#define RADIO_REMOTE_ADDR       "ctrl0"

//-----------------------------------------------------------------

void commConfig(void);

void commPollReceiver(void);
bool commSendPayload(void);

void commProcessData(void);

//-----------------------------------------------------------------

#endif /* COMM_H_ */
