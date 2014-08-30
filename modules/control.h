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

enum control_type
{
    CONTROL_THROTTLE = 0,
    CONTROL_PITCH,
    CONTROL_ROLL,
    CONTROL_YAW,
    CONTROL_PITCH_RATE,
    CONTROL_ROLL_RATE,
    CONTROL_YAW_RATE,

    CONTROL_TYPE_NUM
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

enum axis_mode
{
    AXIS_MODE_DISABLED = 0,
    AXIS_MODE_ANGLE,
    AXIS_MODE_RATE
};

struct control_axis_val
{
    float           value;
    bool            override;
    enum axis_mode  mode;
};

struct control_vals
{
    float throttle;
    struct control_axis_val pitch;
    struct control_axis_val roll;
    struct control_axis_val yaw;
};

//-----------------------------------------------------------------

result_t control_init(void);

void control_arm(void);
void control_disarm(void);
bool control_is_armed(void);

result_t control_set_vals(const struct control_vals *vals);
result_t control_get_vals(struct control_vals *out);
result_t control_set_axis_modes(enum axis_mode pitch, enum axis_mode roll, enum axis_mode yaw);

struct pid* control_get_pid(enum control_type type);

void control_process(void);

//-----------------------------------------------------------------

#endif /* CONTROL_H_ */
