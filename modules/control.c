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

#define CONTROL_THROTTLE_DEAD_ZONE  100.0f
#define CONTROL_PITCH_DEAD_ZONE     100.0f
#define CONTROL_ROLL_DEAD_ZONE      100.0f
#define CONTROL_YAW_DEAD_ZONE       100.0f

#define CONTROL_WATCHDOG_RELOAD     ( SysCtlClockGet() / 2 )

//-----------------------------------------------------------------

static bool armed;
static bool connected;
static struct cmd_control control;

static struct pid pid_pitch;
static struct pid pid_roll;
static struct pid pid_yaw;
static struct pid pid_pitch_rate;
static struct pid pid_roll_rate;
static struct pid pid_yaw_rate;

static struct pid *pid_ptr[] = {
    [PID_PITCH]         = &pid_pitch,
    [PID_ROLL]          = &pid_roll,
    [PID_YAW]           = &pid_yaw,
    [PID_PITCH_RATE]    = &pid_pitch_rate,
    [PID_ROLL_RATE]     = &pid_roll_rate,
    [PID_YAW_RATE]      = &pid_yaw_rate
};

//-----------------------------------------------------------------

static void rcp_cb_angles(struct rcp_msg *msg)
{
    memcpy(&control, &msg->control, sizeof(control));

    if (armed && !control.flags.sw1)
    {
        armed = false;
        motors_disarm();
        buzzer_seq_lib_play(BUZZER_SEQ_DISARM);
    }
    else if (!armed && control.flags.sw1)
    {
        armed = true;
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
            if (!connected)
            {
                buzzer_seq_lib_play(BUZZER_SEQ_CONNECTED);
                connected = true;
            }
        }
        else if (connected)
        {
            buzzer_seq_lib_play(BUZZER_SEQ_LOST);
            connected = false;
            armed = false;
            motors_disarm();
        }

        imu_get_angles(&angles);
        imu_get_rates(&rates);

//        pid_pitch_rate.set_point = 45.0f;

        const float dt = 0.01f;

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

        vTaskDelayUntil(&last_wake, MSEC_TO_TICKS(10));
    }
}

//-----------------------------------------------------------------

result_t control_init(void)
{
    watchdog_init();
    watchdog_reload_set(CONTROL_WATCHDOG_RELOAD);

    armed = false;
    connected = false;

    int i;
    for (i = 0; i < PID_TYPE_NUM; ++i)
        pid_reset(pid_ptr[i]);

    params_get_pid_pitch(&pid_pitch.params);
    params_get_pid_roll(&pid_roll.params);
    params_get_pid_yaw(&pid_yaw.params);
    params_get_pid_pitch_rate(&pid_pitch_rate.params);
    params_get_pid_roll_rate(&pid_roll_rate.params);
    params_get_pid_yaw_rate(&pid_yaw_rate.params);

    rcp_register_callback(RCP_CMD_CONTROL, rcp_cb_angles, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_set, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_get, true);

    daq_register_value("Pitch error", "deg", &pid_pitch.error, DAQ_TYPE_FLOAT);
    daq_register_value("Roll error", "deg", &pid_roll.error, DAQ_TYPE_FLOAT);
    daq_register_value("Yaw error", "deg", &pid_yaw.error, DAQ_TYPE_FLOAT);
    daq_register_value("Pitch control", "deg", &pid_pitch.output, DAQ_TYPE_FLOAT);
    daq_register_value("Roll control", "deg", &pid_roll.output, DAQ_TYPE_FLOAT);
    daq_register_value("Yaw control", "deg", &pid_yaw.output, DAQ_TYPE_FLOAT);
    daq_register_value("Pitch setpoint", "deg", &pid_pitch.setpoint, DAQ_TYPE_FLOAT);
    daq_register_value("Roll setpoint", "deg", &pid_roll.setpoint, DAQ_TYPE_FLOAT);
    daq_register_value("Yaw setpoint", "deg", &pid_yaw.setpoint, DAQ_TYPE_FLOAT);
    daq_register_value("Pitch rate error", "deg/s", &pid_pitch_rate.error, DAQ_TYPE_FLOAT);
    daq_register_value("Roll rate error", "deg/s", &pid_roll_rate.error, DAQ_TYPE_FLOAT);
    daq_register_value("Yaw rate error", "deg/s", &pid_yaw_rate.error, DAQ_TYPE_FLOAT);
    daq_register_value("Pitch rate control", "deg/s", &pid_pitch_rate.output, DAQ_TYPE_FLOAT);
    daq_register_value("Roll rate control", "deg/s", &pid_roll_rate.output, DAQ_TYPE_FLOAT);
    daq_register_value("Yaw rate control", "deg/s", &pid_yaw_rate.output, DAQ_TYPE_FLOAT);
    daq_register_value("Pitch rate setpoint", "deg/s", &pid_pitch_rate.setpoint, DAQ_TYPE_FLOAT);
    daq_register_value("Roll rate setpoint", "deg/s", &pid_roll_rate.setpoint, DAQ_TYPE_FLOAT);
    daq_register_value("Yaw rate setpoint", "deg/s", &pid_yaw_rate.setpoint, DAQ_TYPE_FLOAT);

    if (xTaskCreate(control_task, TASK_NAME("CTRL"),
            CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIORITY, NULL) != pdPASS)
        return RES_ERR_FATAL;

//    watchdog_enable();

    return RES_OK;
}

//-----------------------------------------------------------------

result_t control_get_current(struct cmd_control *out)
{
    memcpy(out, &control, sizeof(control));
    return RES_OK;
}

//-----------------------------------------------------------------

struct pid* control_get_pid(enum pid_type type)
{
    if ((int)type >= (int)PID_TYPE_NUM)
        return NULL;

    return pid_ptr[(int)type];
}
