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
#include <FreeRTOS.h>
#include <semphr.h>
#include <string.h>

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

struct pid_axis
{
    struct pid angle;
    struct pid rate;
};

struct control_obj
{
    bool armed;
    bool connected;

    struct pid_axis pid_pitch;
    struct pid_axis pid_roll;
    struct pid_axis pid_yaw;

    struct control_vals curr_control;

    xSemaphoreHandle mutex;
};

//-----------------------------------------------------------------

static struct control_obj control;

static struct pid *pid_ptr[CONTROL_TYPE_NUM] = {
    [CONTROL_THROTTLE]      = NULL,
    [CONTROL_PITCH]         = &control.pid_pitch.angle,
    [CONTROL_ROLL]          = &control.pid_roll.angle,
    [CONTROL_YAW]           = &control.pid_yaw.angle,
    [CONTROL_PITCH_RATE]    = &control.pid_pitch.rate,
    [CONTROL_ROLL_RATE]     = &control.pid_roll.rate,
    [CONTROL_YAW_RATE]      = &control.pid_yaw.rate
};

//-----------------------------------------------------------------

static void control_task(void *params);

static void control_lock(void);
static void control_unlock(void);

static void rcp_cb_control(struct rcp_msg *msg);
static void rcp_cb_pid_set(struct rcp_msg *msg);
static void rcp_cb_pid_get(struct rcp_msg *msg);
static void rcp_cb_limits_set(struct rcp_msg *msg);
static void rcp_cb_limits_get(struct rcp_msg *msg);

static result_t calc_control(const struct cmd_control *cmd, struct control_vals *out);
static result_t calc_axis_control(const struct control_axis *cmd, struct control_axis_val *out,
        enum control_type type_angle, enum control_type type_rate);
static result_t limit_control(enum control_type type, bool absolute, bool bipolar, float val, float *out);

//-----------------------------------------------------------------

result_t control_init(void)
{
    watchdog_init();
    watchdog_reload_set(CONTROL_WATCHDOG_RELOAD);

    control.armed = false;
    control.connected = false;
    memset(&control.curr_control, 0, sizeof(control.curr_control));

    int i;
    for (i = 0; i < CONTROL_TYPE_NUM; ++i)
        pid_reset(pid_ptr[i]);

    params_get_pid_pitch(&control.pid_pitch.angle.params);
    params_get_pid_roll(&control.pid_roll.angle.params);
    params_get_pid_yaw(&control.pid_yaw.angle.params);
    params_get_pid_pitch_rate(&control.pid_pitch.rate.params);
    params_get_pid_roll_rate(&control.pid_roll.rate.params);
    params_get_pid_yaw_rate(&control.pid_yaw.rate.params);

    rcp_register_callback(RCP_CMD_CONTROL, rcp_cb_control, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_set, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_get, true);
    rcp_register_callback(RCP_CMD_LIMITS, rcp_cb_limits_set, false);
    rcp_register_callback(RCP_CMD_LIMITS, rcp_cb_limits_get, true);

    DAQ_REGISTER_PID_AXIS("Pitch", control.pid_pitch);
    DAQ_REGISTER_PID_AXIS("Roll", control.pid_roll);
    DAQ_REGISTER_PID_AXIS("Yaw", control.pid_yaw);

    control.mutex = xSemaphoreCreateRecursiveMutex();

    if (xTaskCreate(control_task, TASK_NAME("CTRL"),
            CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIORITY, NULL) != pdPASS)
        return RES_ERR_FATAL;

//    watchdog_enable();

    return RES_OK;
}

//-----------------------------------------------------------------

void control_process(void)
{
    control_lock();

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

    struct vec3 angles, rates;
    imu_get_angles(&angles);
    imu_get_rates(&rates);

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

    control_unlock();
}

//-----------------------------------------------------------------

static void control_task(void *params)
{
    portTickType last_wake = xTaskGetTickCount();

    while (1)
    {
        control_process();
        vTaskDelayUntil(&last_wake, MSEC_TO_TICKS(10));
    }
}

//-----------------------------------------------------------------

void control_arm(void)
{
    control_lock();
    control.armed = true;
    motors_arm();
    control_unlock();
    buzzer_seq_lib_play(BUZZER_SEQ_ARM);
}

//-----------------------------------------------------------------

void control_disarm(void)
{
    control_lock();
    control.armed = false;
    motors_disarm();
    control_unlock();
    buzzer_seq_lib_play(BUZZER_SEQ_DISARM);
}

//-----------------------------------------------------------------

bool control_is_armed(void)
{
    return control.armed;
}

//-----------------------------------------------------------------

result_t control_set_vals(const struct control_vals *vals)
{
    if (!vals)
        return RES_ERR_BAD_PARAM;

    control_lock();
    memcpy(&control.curr_control, vals, sizeof(control.curr_control));
    control_unlock();
    return RES_OK;
}

//-----------------------------------------------------------------

result_t control_get_vals(struct control_vals *out)
{
    if (!out)
        return RES_ERR_BAD_PARAM;

    control_lock();
    memcpy(out, &control.curr_control, sizeof(control.curr_control));
    control_unlock();
    return RES_OK;
}

//-----------------------------------------------------------------

struct pid* control_get_pid(enum control_type type)
{
    if ((int)type >= (int)CONTROL_TYPE_NUM)
        return NULL;

    return pid_ptr[(int)type];
}

//-----------------------------------------------------------------

static void control_lock(void)
{
    xSemaphoreTakeRecursive(control.mutex, portMAX_DELAY);
}

//-----------------------------------------------------------------

static void control_unlock(void)
{
    xSemaphoreGiveRecursive(control.mutex);
}

//-----------------------------------------------------------------

static result_t calc_control(const struct cmd_control *cmd, struct control_vals *out)
{
    if (!cmd || !out)
        return RES_ERR_BAD_PARAM;

    result_t res;
    res = limit_control(CONTROL_THROTTLE, cmd->throttle.absolute, false, cmd->throttle.value, &out->throttle);
    if (res != RES_OK)
        return res;

    res = calc_axis_control(&cmd->pitch, &out->pitch, CONTROL_PITCH, CONTROL_PITCH_RATE);
    if (res != RES_OK)
        return res;
    res = calc_axis_control(&cmd->roll, &out->roll, CONTROL_ROLL, CONTROL_ROLL_RATE);
    if (res != RES_OK)
        return res;
    res = calc_axis_control(&cmd->yaw, &out->yaw, CONTROL_YAW, CONTROL_YAW_RATE);
    if (res != RES_OK)
        return res;

    return RES_OK;
}

//-----------------------------------------------------------------

static result_t calc_axis_control(const struct control_axis *cmd, struct control_axis_val *out,
        enum control_type type_angle, enum control_type type_rate)
{
    if (!cmd || !out)
        return RES_ERR_BAD_PARAM;

    out->mode = (enum axis_mode)cmd->mode;
    enum control_type type;
    switch (out->mode)
    {
    case AXIS_MODE_ANGLE:
        type = type_angle;
        break;
    case AXIS_MODE_RATE:
        type = type_rate;
        break;
    default:
        out->value = 0.0f;
        return RES_OK;
    }

    return limit_control(type, cmd->value.absolute, true, cmd->value.value, &out->value);
}

//-----------------------------------------------------------------

static result_t limit_control(enum control_type type, bool absolute, bool bipolar, float val, float *out)
{
    if (!out)
        return RES_ERR_BAD_PARAM;

    struct control_limit limit;
    result_t res = params_get_limit(type, &limit);
    if (res != RES_OK)
        return RES_ERR_FATAL;

    if (!absolute)
        val *= limit.limit;

    if (val > limit.limit)
        val = limit.limit;
    else if (bipolar && (val < -limit.limit))
        val = -limit.limit;
    else if (!bipolar && (val < 0.0f))
        val = 0.0f;

    if (fabsf(val) < limit.dead_zone)
        val = 0.0f;

    *out = val;
    return RES_OK;
}

//-----------------------------------------------------------------

static void rcp_cb_control(struct rcp_msg *msg)
{
    control_lock();

    struct cmd_control *cmd = &msg->control;

    if (control.armed && !cmd->flags.armed)
        control_disarm();
    else if (!control.armed && cmd->flags.armed)
        control_arm();

    struct control_vals vals;
    if (calc_control(cmd, &vals) == RES_OK)
        control_set_vals(&vals);

    control_unlock();
}

//-----------------------------------------------------------------

static void rcp_cb_pid_set(struct rcp_msg *msg)
{
    control_lock();

    struct pid *pid = pid_ptr[msg->pid.type];

    pid->params.kp = msg->pid.kp;
    pid->params.kd = msg->pid.kd;
    pid->params.ki = msg->pid.ki;
    pid->params.kt = msg->pid.kt;

    pid_reset(pid);

    control_unlock();

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

static void rcp_cb_limits_set(struct rcp_msg *msg)
{
    control_lock();

    uint8_t i;
    for (i = 0; i < LIMITS_MAX; ++i)
    {
        const struct limit_data *arg = &msg->limits.limits[i];
        if (arg->valid)
        {
            struct control_limit par;
            result_t res = params_get_limit((enum control_type)arg->type, &par);
            if (res == RES_OK)
            {
                struct control_limit data = {
                    .limit      = arg->limit.limit,
                    .dead_zone  = arg->limit.dead_zone
                };
                params_set_limit((enum control_type)arg->type, &data);
            }
        }
    }

    control_unlock();

    buzzer_seq_lib_play(BUZZER_SEQ_CONFIRM);
}

//-----------------------------------------------------------------

static void rcp_cb_limits_get(struct rcp_msg *msg)
{
    struct rcp_msg resp;
    resp.cmd    = RCP_CMD_LIMITS;
    resp.query  = RCP_CMD_OK;

    memset(&resp.limits, 0, sizeof(resp.limits));

    uint8_t i;
    for (i = 0; i < LIMITS_MAX; ++i)
    {
        const struct limit_data *arg = &msg->limits.limits[i];
        if (arg->valid)
        {
            struct control_limit par;
            result_t res = params_get_limit((enum control_type)arg->type, &par);
            if (res == RES_OK)
            {
                struct limit_data *dst = &resp.limits.limits[i];
                dst->valid = true;
                dst->type = arg->type;
                dst->limit.limit = par.limit;
                dst->limit.dead_zone = par.dead_zone;
            }
        }
    }

    rcp_send_message(&resp);
}
