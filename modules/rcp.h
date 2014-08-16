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
#include "rcp_cmd.h"

//-----------------------------------------------------------------

typedef void (*rcp_callback_t)(struct rcp_msg*);

//-----------------------------------------------------------------

void rcp_init(void);

bool rcp_send_message(struct rcp_msg *msg);
void rcp_register_callback(enum rcp_cmd cmd, rcp_callback_t callback, bool query);

void rcp_enable_rx(void);
void rcp_enable_tx(void);

void rcp_process_message(struct rcp_msg *msg);

bool rcp_is_connected(void);

//-----------------------------------------------------------------

#endif /* RCP_H_ */
