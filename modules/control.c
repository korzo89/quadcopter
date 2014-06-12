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
#include <utils/delay.h>
#include <utils/pid.h>
#include <utils/buzzer_seq.h>

#include <stellaris_config.h>

//-----------------------------------------------------------------

#define CONTROL_TASK_STACK          200

#define CONTROL_THROTTLE_DEAD_ZONE  100.0f
#define CONTROL_PITCH_DEAD_ZONE     100.0f
#define CONTROL_ROLL_DEAD_ZONE      100.0f
#define CONTROL_YAW_DEAD_ZONE       100.0f

#define CONTROL_WATCHDOG_RELOAD     ( SysCtlClockGet() / 2 )

//-----------------------------------------------------------------

static bool armed;
static bool connected;
static control_t control;

//-----------------------------------------------------------------

typedef struct PACK_STRUCT
{
    uint8_t type;
    float kp;
    float ki;
    float kd;
} cmd_pid_data_t;

//-----------------------------------------------------------------

static pid_t pid_pitch = {
    .kp         = 1.0f,
    .ki         = 0.0f,
    .kd         = 0.0f,
    .kt         = 0.0f,
    .int_max    = 1000.0f,
    .int_min    = -1000.0f
};
static pid_t pid_roll = {
    .kp         = 1.0f,
    .ki         = 0.0f,
    .kd         = 0.0f,
    .kt         = 0.0f,
    .int_max    = 1000.0f,
    .int_min    = -1000.0f
};
static pid_t pid_yaw = {
    .kp         = 1.0f,
    .ki         = 0.0f,
    .kd         = 0.0f,
    .kt         = 0.0f,
    .int_max    = 1000.0f,
    .int_min    = -1000.0f
};
static pid_t pid_pitch_rate = {
    .kp         = 1.0f,
    .ki         = 0.0f,
    .kd         = 0.0f,
    .kt         = 0.0f,
    .int_max    = 1000.0f,
    .int_min    = -1000.0f
};
static pid_t pid_roll_rate = {
    .kp         = 1.0f,
    .ki         = 0.0f,
    .kd         = 0.0f,
    .kt         = 0.0f,
    .int_max    = 1000.0f,
    .int_min    = -1000.0f
};
static pid_t pid_yaw_rate = {
    .kp         = 1.0f,
    .ki         = 0.0f,
    .kd         = 0.0f,
    .kt         = 0.0f,
    .int_max    = 1000.0f,
    .int_min    = -1000.0f
};

static pid_t *pid_ptr[] = {
    [PID_PITCH]         = &pid_pitch,
    [PID_ROLL]          = &pid_roll,
    [PID_YAW]           = &pid_yaw,
    [PID_PITCH_RATE]    = &pid_pitch_rate,
    [PID_ROLL_RATE]     = &pid_roll_rate,
    [PID_YAW_RATE]      = &pid_yaw_rate
};

//-----------------------------------------------------------------

static void rcp_cb_angles(rcp_message_t *msg)
{
    memcpy((uint8_t*)&control, msg->packet.data, sizeof(control_t));

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

static void rcp_cb_pid_set(rcp_message_t *msg)
{
    cmd_pid_data_t *data = (cmd_pid_data_t*)msg->packet.data;

    pid_t *pid = pid_ptr[data->type];

    pid->kp = data->kp;
    pid->kd = data->kd;
    pid->ki = data->ki;

    pid_reset(pid);

    buzzer_seq_lib_play(BUZZER_SEQ_CONFIRM);
}

//-----------------------------------------------------------------

static void rcp_cb_pid_get(rcp_message_t *msg)
{
    rcp_message_t resp;
    resp.packet.cmd = RCP_CMD_PID;
    resp.packet.query = RCP_CMD_OK;

    uint8_t type = ((cmd_pid_data_t*)msg->packet.data)->type;

    pid_t *pid = pid_ptr[(int)type];

    cmd_pid_data_t data;
    data.type = type;
    data.kp = pid->kp;
    data.ki = pid->ki;
    data.kd = pid->kd;

    memcpy(resp.packet.data, (uint8_t*)&data, sizeof(data));

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void control_task(void *params)
{
    portTickType last_wake = xTaskGetTickCount();

    vec3_t angles, rates;

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

//        const float dt = 0.01f;

        if (armed)
        {
            float throttle = (float)control.throttle;
            if (throttle < CONTROL_THROTTLE_DEAD_ZONE)
                throttle = 0.0f;
            throttle = throttle / 4095.0 * THROTTLE_MAX;

//            pid_update_auto(&pid_pitch_rate, angles.y, dt);

//            throttle = pid_pitch_rate.output;

            motors_set_throttle(throttle, throttle, throttle, throttle);
        }
//        else
//        {
//            pid_update_manual(&pid_pitch_rate, angles.y, dt, 0.0f);
//        }

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

    rcp_register_callback(RCP_CMD_CONTROL, rcp_cb_angles, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_set, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_get, true);

    if (xTaskCreate(control_task, (signed portCHAR*)"CTRL",
            CONTROL_TASK_STACK, NULL, 2, NULL) != pdPASS)
        return RES_ERR_FATAL;

    watchdog_enable();

    return RES_OK;
}

//-----------------------------------------------------------------

result_t control_get_current(control_t *out)
{
    memcpy(out, &control, sizeof(control_t));
    return RES_OK;
}

//-----------------------------------------------------------------

pid_t* control_get_pid(pid_type_t type)
{
    if ((int)type >= (int)PID_TYPE_NUM)
        return NULL;

    return pid_ptr[(int)type];
}
