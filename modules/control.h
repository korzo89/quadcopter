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

struct control_limit
{
    float limit;
    float dead_zone;
};

struct control_limit_axes
{
    struct control_limit pitch;
    struct control_limit roll;
    struct control_limit yaw;
};

//-----------------------------------------------------------------

result_t control_init(void);

bool control_is_armed(void);

result_t control_get_current(struct cmd_control *out);

struct pid* control_get_pid(enum pid_type type);

//-----------------------------------------------------------------

#endif /* CONTROL_H_ */
