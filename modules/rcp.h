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

#define IS_VALID_CMD(x)         ( (int)(x) >= (int)RCP_CMD_OK && (int)(x) < (int)RCP_CMD_NUM )

//-----------------------------------------------------------------

typedef enum
{
    RCP_CMD_OK          = 0x00,
    RCP_CMD_ERROR       = 0x01,
    RCP_CMD_CONTROL     = 0x02,
    RCP_CMD_RAW_IMU		= 0x03,
    RCP_CMD_ANGLES      = 0x04,
    RCP_CMD_PID         = 0x05,

    RCP_CMD_PARAM_LIST  = 0x06,
    RCP_CMD_PARAM_INFO  = 0x07,
    RCP_CMD_PARAM_GET   = 0x08,
    RCP_CMD_PARAM_SET   = 0x09,
    RCP_CMD_PARAM_SAVE  = 0x0A,

    RCP_CMD_NUM
} rcp_command_t;

typedef struct PACK_STRUCT
{
    uint8_t cmd;
    uint8_t query;
    uint8_t data[RCP_PAYLOAD_SIZE - 2];
} rcp_packet_t;

typedef union
{
    uint8_t raw[RCP_PAYLOAD_SIZE];
    rcp_packet_t packet;
} rcp_message_t;

typedef void (*rcp_callback_t)(rcp_message_t*);

//-----------------------------------------------------------------

void rcp_init(void);

bool rcp_send_message(rcp_message_t *msg);
void rcp_register_callback(rcp_command_t cmd, rcp_callback_t callback, bool query);

void rcp_enable_rx(void);
void rcp_enable_tx(void);

void rcp_process_message(rcp_message_t *msg);

bool rcp_is_connected(void);

//-----------------------------------------------------------------

#endif /* RCP_H_ */
