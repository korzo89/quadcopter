/*
 * comm.h
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#ifndef COMM_H_
#define COMM_H_

//-----------------------------------------------------------------

#define PAYLOAD_SIZE            24

#define RADIO_LOCAL_ADDR        "serv1"
#define RADIO_REMOTE_ADDR       "clie1"

#define MSG_START               0x99
#define MSG_SET_MOTORS          0x01

//-----------------------------------------------------------------

void commConfig(void);

void commPollReceiver(void);

void commProcessData(void);

//-----------------------------------------------------------------

#endif /* COMM_H_ */
