/*
 * control.h
 *
 *  Created on: 28 maj 2014
 *      Author: Korzo
 */

#ifndef CONTROL_H_
#define CONTROL_H_

//-----------------------------------------------------------------

#include <defs.h>
#include <utils/pid.h>
#include <modules/rcp_cmd.h>

//-----------------------------------------------------------------

enum pid_type
{
    PID_PITCH = 0,
    PID_ROLL,
    PID_YAW,
    PID_PITCH_RATE,
    PID_ROLL_RATE,
    PID_YAW_RATE,

    PID_TYPE_NUM
};

//-----------------------------------------------------------------

result_t control_init(void);

result_t control_get_current(struct cmd_control *out);

struct pid* control_get_pid(enum pid_type type);

//-----------------------------------------------------------------

#endif /* CONTROL_H_ */
