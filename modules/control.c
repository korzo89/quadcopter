/*
 * control.c
 *
 *  Created on: 28 maj 2014
 *      Author: Korzo
 */

#include "control.h"

#include <drivers/buzzer.h>
#include <drivers/motors.h>
#include <drivers/watchdog.h>
#include <modules/rcp.h>
#include <modules/imu.h>
#include <modules/daq.h>
#include <modules/params.h>
#include <utils/delay.h>
#include <utils/pid.h>
#include <utils/buzzer_seq.h>

#include <stellaris_config.h>
#include <utils/ustdlib.h>

//-----------------------------------------------------------------

#define CONTROL_THROTTLE_DEAD_ZONE      100.0f
#define CONTROL_PITCH_DEAD_ZONE         100.0f
#define CONTROL_ROLL_DEAD_ZONE          100.0f
#define CONTROL_YAW_DEAD_ZONE           100.0f

#define CONTROL_WATCHDOG_RELOAD         ( SysCtlClockGet() / 2 )

#define DAQ_REGISTER_PID(_name, _unit, _pid)   \
    do {                                                                                \
        daq_register_value(_name" error", _unit, &_pid.error, DAQ_TYPE_FLOAT);          \
        daq_register_value(_name" control", _unit, &_pid.output, DAQ_TYPE_FLOAT);       \
        daq_register_value(_name" setpoint", _unit, &_pid.setpoint, DAQ_TYPE_FLOAT);    \
    } while(0)

#define DAQ_REGISTER_PID_AXIS(_name, _pid)   \
    do {                                                        \
        DAQ_REGISTER_PID(_name, "deg", _pid.angle);             \
        DAQ_REGISTER_PID(_name" rate", "deg/s", _pid.rate);    \
    } while (0)

//-----------------------------------------------------------------

enum pid_mode
{
    PID_MODE_DISABLED = 0,
    PID_MODE_ANGLE,
    PID_MODE_RATE
};

struct pid_axis
{
    enum pid_mode   mode;
    struct pid      angle;
    struct pid      rate;
};

struct control_obj
{
    bool armed;
    bool connected;

    struct pid_axis pid_pitch;
    struct pid_axis pid_roll;
    struct pid_axis pid_yaw;

    struct cmd_control cmd;
};

//-----------------------------------------------------------------

static struct control_obj control;

static struct pid *pid_ptr[] = {
    [PID_PITCH]         = &control.pid_pitch.angle,
    [PID_ROLL]          = &control.pid_roll.angle,
    [PID_YAW]           = &control.pid_yaw.angle,
    [PID_PITCH_RATE]    = &control.pid_pitch.rate,
    [PID_ROLL_RATE]     = &control.pid_roll.rate,
    [PID_YAW_RATE]      = &control.pid_yaw.rate
};

//-----------------------------------------------------------------

static void rcp_cb_angles(struct rcp_msg *msg)
{
    memcpy(&control.cmd, &msg->control, sizeof(control.cmd));

    if (control.armed && !control.cmd.flags.armed)
    {
        control.armed = false;
        motors_disarm();
        buzzer_seq_lib_play(BUZZER_SEQ_DISARM);
    }
    else if (!control.armed && control.cmd.flags.armed)
    {
        control.armed = true;
        motors_arm();
        buzzer_seq_lib_play(BUZZER_SEQ_ARM);
    }
}

//-----------------------------------------------------------------

static void rcp_cb_pid_set(struct rcp_msg *msg)
{
    struct pid *pid = pid_ptr[msg->pid.type];

    pid->params.kp = msg->pid.kp;
    pid->params.kd = msg->pid.kd;
    pid->params.ki = msg->pid.ki;
    pid->params.kt = msg->pid.kt;

    pid_reset(pid);

    buzzer_seq_lib_play(BUZZER_SEQ_CONFIRM);
}

//-----------------------------------------------------------------

static void rcp_cb_pid_get(struct rcp_msg *msg)
{
    struct rcp_msg resp;
    resp.cmd    = RCP_CMD_PID;
    resp.query  = RCP_CMD_OK;

    uint8_t type = msg->pid.type;
    struct pid *pid = pid_ptr[(int)type];

    resp.pid.type = type;
    resp.pid.kp = pid->params.kp;
    resp.pid.ki = pid->params.ki;
    resp.pid.kd = pid->params.kd;
    resp.pid.kt = pid->params.kt;

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void control_task(void *params)
{
    portTickType last_wake = xTaskGetTickCount();

    struct vec3 angles, rates;

    while (1)
    {
        watchdog_clear();

        if (rcp_is_connected())
        {
            if (!control.connected)
            {
                buzzer_seq_lib_play(BUZZER_SEQ_CONNECTED);
                control.connected = true;
            }
        }
        else if (control.connected)
        {
            buzzer_seq_lib_play(BUZZER_SEQ_LOST);
            control.connected = false;
            control.armed = false;
            motors_disarm();
        }

        imu_get_angles(&angles);
        imu_get_rates(&rates);

//        pid_pitch_rate.set_point = 45.0f;

        const float dt = 0.01f;

#if 0
        if (armed)
        {
            float throttle = (float)control.throttle;
            if (throttle < CONTROL_THROTTLE_DEAD_ZONE)
                throttle = 0.0f;
            throttle = throttle / 4095.0 * THROTTLE_MAX;

//            pid_update_auto(&pid_pitch_rate, angles.y, dt);

//            throttle = pid_pitch_rate.output;

            if (throttle <= 100.0f)
            {
                motors_set_throttle(throttle, throttle, throttle, throttle);
//                motors_set_throttle(0, 0, 0, 0);
                pid_update_manual(&pid_pitch_rate, rates.y, dt, 0.0f);
            }
            else
            {
                float sp = (float)control.pitch - 4095.0f / 2.0f;
                if (fabs(sp) < CONTROL_PITCH_DEAD_ZONE)
                    sp = 0.0f;
                else
                    sp = sp / 4095.0f * 100.0f;

                pid_pitch_rate.setpoint = sp;
                pid_update_auto(&pid_pitch_rate, rates.y, dt);

                float pitch_out = pid_pitch_rate.output;

                motors_set_throttle(throttle - pitch_out, throttle + pitch_out,
                        throttle + pitch_out, throttle - pitch_out);
//                motors_set_throttle(-pitch_out, pitch_out, pitch_out, -pitch_out);
            }
        }
        else
        {
            pid_update_manual(&pid_pitch_rate, rates.y, dt, 0.0f);
        }
#endif

        vTaskDelayUntil(&last_wake, MSEC_TO_TICKS(10));
    }
}

//-----------------------------------------------------------------

result_t control_init(void)
{
    watchdog_init();
    watchdog_reload_set(CONTROL_WATCHDOG_RELOAD);

    control.armed = false;
    control.connected = false;

    int i;
    for (i = 0; i < PID_TYPE_NUM; ++i)
        pid_reset(pid_ptr[i]);

    params_get_pid_pitch(&control.pid_pitch.angle.params);
    params_get_pid_roll(&control.pid_roll.angle.params);
    params_get_pid_yaw(&control.pid_yaw.angle.params);
    params_get_pid_pitch_rate(&control.pid_pitch.rate.params);
    params_get_pid_roll_rate(&control.pid_roll.rate.params);
    params_get_pid_yaw_rate(&control.pid_yaw.rate.params);

    rcp_register_callback(RCP_CMD_CONTROL, rcp_cb_angles, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_set, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_get, true);

    DAQ_REGISTER_PID_AXIS("Pitch", control.pid_pitch);
    DAQ_REGISTER_PID_AXIS("Roll", control.pid_roll);
    DAQ_REGISTER_PID_AXIS("Yaw", control.pid_yaw);

    if (xTaskCreate(control_task, TASK_NAME("CTRL"),
            CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIORITY, NULL) != pdPASS)
        return RES_ERR_FATAL;

//    watchdog_enable();

    return RES_OK;
}

//-----------------------------------------------------------------

result_t control_get_current(struct cmd_control *out)
{
    memcpy(out, &control.cmd, sizeof(control.cmd));
    return RES_OK;
}

//-----------------------------------------------------------------

struct pid* control_get_pid(enum pid_type type)
{
    if ((int)type >= (int)PID_TYPE_NUM)
        return NULL;

    return pid_ptr[(int)type];
}
