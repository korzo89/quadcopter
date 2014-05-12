/*
 * rcp.h
 *
 *  Created on: 09-10-2013
 *      Author: Korzo
 */

#ifndef RCP_H_
#define RCP_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

#define RCP_PAYLOAD_SIZE        32

//-----------------------------------------------------------------

typedef enum
{
    RCP_CMD_OK          = 0x00,
    RCP_CMD_CONTROL     = 0x01,

    RCP_CMD_NUM
} RCPCommand;

typedef union
{
    uint8_t raw[RCP_PAYLOAD_SIZE];

    struct PACK_STRUCT
    {
        uint8_t cmd;
        uint8_t query;
        uint8_t data[RCP_PAYLOAD_SIZE - 2];
    };
} RCPMessage;

typedef void (*RCPCallback)(RCPMessage*);

//-----------------------------------------------------------------

void rcpInit(void);

void rcpSendMessage(RCPMessage *msg);
void rcpRegisterCallback(RCPCommand cmd, RCPCallback callback);

void rcpEnableRx(void);
void rcpEnableTx(void);

//-----------------------------------------------------------------

#endif /* RCP_H_ */
