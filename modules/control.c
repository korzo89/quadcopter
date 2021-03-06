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
#include <drivers/led.h>
#include <modules/rcp.h>
#include <modules/imu.h>
#include <modules/daq.h>
#include <modules/params.h>
#include <utils/delay.h>
#include <utils/pid.h>
#include <utils/buzzer_seq.h>
#include <utils/tickstamp.h>

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

#define DAQ_REGISTER_PID(_name, _unit, _pid)                                            \
    do {                                                                                \
        daq_register_value(_name" error", _unit, &_pid.error, DAQ_TYPE_FLOAT);          \
        daq_register_value(_name" control", _unit, &_pid.output, DAQ_TYPE_FLOAT);       \
        daq_register_value(_name" setpoint", _unit, &_pid.setpoint, DAQ_TYPE_FLOAT);    \
    } while(0)

#define DAQ_REGISTER_PID_AXIS(_name, _pid)                      \
    do {                                                        \
        DAQ_REGISTER_PID(_name, "deg", _pid.angle);             \
        DAQ_REGISTER_PID(_name" rate", "deg/s", _pid.rate);     \
    } while (0)

#define DAQ_REGISTER_MOTOR(_num, _ptr)  \
    daq_register_value("Motor " #_num " control", "raw", _ptr, DAQ_TYPE_FLOAT)

#define DAQ_REGISTER_MOTORS(_mot)     \
    do {                                    \
        DAQ_REGISTER_MOTOR(1, &_mot.m1);    \
        DAQ_REGISTER_MOTOR(2, &_mot.m2);    \
        DAQ_REGISTER_MOTOR(3, &_mot.m3);    \
        DAQ_REGISTER_MOTOR(4, &_mot.m4);    \
    } while (0)

//-----------------------------------------------------------------

struct pid_axis
{
    enum axis_mode mode;
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

    struct control_vals     curr_control;
    struct control_motors   motors;

    bool override_motors;
    struct control_motors override_vals;

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
static void rcp_cb_axis_modes_set(struct rcp_msg *msg);
static void rcp_cb_axis_modes_get(struct rcp_msg *msg);
static void rcp_cb_motors(struct rcp_msg *msg);

static result_t calc_control(const struct cmd_control *cmd, struct control_vals *out);
static result_t calc_axis_control(const struct control_axis *cmd, struct control_axis_val *out,
        const struct pid_axis *pid, enum control_type type_angle, enum control_type type_rate);
static result_t limit_control(enum control_type type, bool absolute, bool bipolar, bool dead, float val, float *out);

static float update_axis_control(struct pid_axis *axis, float val, float angle, float rate, float dt);

static void set_motors(const struct control_motors *val);
static float limit_motor_val(float val, float max);

static void process_arming(bool arm);

//-----------------------------------------------------------------

result_t control_init(void)
{
    watchdog_init();
    watchdog_reload_set(CONTROL_WATCHDOG_RELOAD);

    memset(&control, 0, sizeof(control));

    uint32_t i;
    for (i = 0; i < CONTROL_TYPE_NUM; ++i)
        pid_reset(pid_ptr[i]);

    params_get_pid_pitch(&control.pid_pitch.angle.params);
    params_get_pid_roll(&control.pid_roll.angle.params);
    params_get_pid_yaw(&control.pid_yaw.angle.params);
    params_get_pid_pitch_rate(&control.pid_pitch.rate.params);
    params_get_pid_roll_rate(&control.pid_roll.rate.params);
    params_get_pid_yaw_rate(&control.pid_yaw.rate.params);

    params_get_pitch_mode(&control.pid_pitch.mode);
    params_get_roll_mode(&control.pid_roll.mode);
    params_get_yaw_mode(&control.pid_yaw.mode);

    rcp_register_callback(RCP_CMD_CONTROL, rcp_cb_control, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_set, false);
    rcp_register_callback(RCP_CMD_PID, rcp_cb_pid_get, true);
    rcp_register_callback(RCP_CMD_LIMITS, rcp_cb_limits_set, false);
    rcp_register_callback(RCP_CMD_LIMITS, rcp_cb_limits_get, true);
    rcp_register_callback(RCP_CMD_AXIS_MODES, rcp_cb_axis_modes_set, false);
    rcp_register_callback(RCP_CMD_AXIS_MODES, rcp_cb_axis_modes_get, true);
    rcp_register_callback(RCP_CMD_MOTORS, rcp_cb_motors, false);

    DAQ_REGISTER_PID_AXIS("Pitch", control.pid_pitch);
    DAQ_REGISTER_PID_AXIS("Roll", control.pid_roll);
    DAQ_REGISTER_PID_AXIS("Yaw", control.pid_yaw);

    DAQ_REGISTER_MOTORS(control.motors);

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
            buzzer_seq_lib_play(BUZZER_SEQ_CONNECTED, BUZZER_MODE_FORCE);
            control.connected = true;
        }
    }
    else if (control.connected)
    {
        buzzer_seq_lib_play(BUZZER_SEQ_LOST, BUZZER_MODE_FORCE);
        control.connected = false;
        control_disarm();
    }

    struct vec3 angles, rates;
    imu_get_angles(&angles);
    imu_get_rates(&rates);

    static uint32_t prev_time = 0;
    uint32_t now = tickstamp_get_systick();
    float dt = ((float)now - (float)prev_time) / 1000.0f;
    prev_time = now;

    struct control_motors motors = {
        .m1 = 0.0f,
        .m2 = 0.0f,
        .m3 = 0.0f,
        .m4 = 0.0f
    };

    bool manual_all = false;
    if (control.armed)
    {
        if (control.override_motors)
        {
            motors = control.override_vals;
            manual_all = true;
        }
        else
        {
            float control_min;
            params_get_control_min_throttle(&control_min);

            float throttle = control.curr_control.throttle;
            if (throttle < control_min)
            {
                manual_all = true;

                motors.m1 = throttle;
                motors.m2 = throttle;
                motors.m3 = throttle;
                motors.m4 = throttle;
            }
            else
            {
                float motor_max;
                params_get_motor_max(&motor_max);

                float pitch = update_axis_control(&control.pid_pitch, control.curr_control.pitch.value, angles.y, rates.y, dt);
                float roll  = update_axis_control(&control.pid_roll, control.curr_control.roll.value, angles.x, rates.x, dt);
                float yaw   = update_axis_control(&control.pid_yaw, control.curr_control.yaw.value, angles.z, rates.z, dt);

                motors.m1 = throttle - pitch + roll + yaw;
                motors.m2 = throttle + pitch + roll - yaw;
                motors.m3 = throttle + pitch - roll + yaw;
                motors.m4 = throttle - pitch - roll - yaw;
            }
        }
    }
    else
    {
        manual_all = true;
    }

    set_motors(&motors);

    if (manual_all)
    {
        pid_update_manual(&control.pid_pitch.angle, angles.y, dt, 0.0f);
        pid_update_manual(&control.pid_pitch.rate, rates.y, dt, 0.0f);
        pid_update_manual(&control.pid_roll.angle, angles.x, dt, 0.0f);
        pid_update_manual(&control.pid_roll.rate, rates.x, dt, 0.0f);
        pid_update_manual(&control.pid_yaw.angle, angles.z, dt, 0.0f);
        pid_update_manual(&control.pid_yaw.rate, rates.z, dt, 0.0f);
    }

    control_unlock();
}

//-----------------------------------------------------------------

static void set_motors(const struct control_motors *val)
{
    float motor_max;
    params_get_motor_max(&motor_max);

    control.motors.m1 = limit_motor_val(val->m1, motor_max);
    control.motors.m2 = limit_motor_val(val->m2, motor_max);
    control.motors.m3 = limit_motor_val(val->m3, motor_max);
    control.motors.m4 = limit_motor_val(val->m4, motor_max);

    motors_set_throttle(control.motors.m1, control.motors.m2,
            control.motors.m3, control.motors.m4);
}

//-----------------------------------------------------------------

static float limit_motor_val(float val, float max)
{
    if (val < 0.0f)
        return 0.0f;
    else if (val > max)
        return max;
    else
        return val;
}

//-----------------------------------------------------------------

static float update_axis_control(struct pid_axis *axis, float val, float angle, float rate, float dt)
{
    switch (axis->mode)
    {
    case AXIS_MODE_ANGLE:
        axis->angle.setpoint = val;
        pid_update_auto(&axis->angle, angle, dt);

        axis->rate.setpoint = axis->angle.output;
        pid_update_auto(&axis->rate, rate, dt);

        return axis->rate.output;

    case AXIS_MODE_RATE:
        pid_update_manual(&axis->angle, angle, dt, val);

        axis->rate.setpoint = val;
        pid_update_auto(&axis->rate, rate, dt);

        return axis->rate.output;

    default:
        pid_update_manual(&axis->angle, angle, dt, 0.0f);
        pid_update_manual(&axis->rate, rate, dt, 0.0f);

        return 0.0f;
    }
}

//-----------------------------------------------------------------

result_t control_get_motors(struct control_motors *out)
{
    if (!out)
        return RES_ERR_BAD_PARAM;
    memcpy(out, &control.motors, sizeof(control.motors));
    return RES_OK;
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

    led_turn_on(LED_GREEN);

    control_unlock();

    buzzer_seq_lib_play(BUZZER_SEQ_ARM, BUZZER_MODE_FORCE);
}

//-----------------------------------------------------------------

void control_disarm(void)
{
    control_lock();

    if (control.armed)
        buzzer_seq_lib_play(BUZZER_SEQ_DISARM, BUZZER_MODE_FORCE);

    control.armed = false;
    motors_disarm();

    led_turn_off(LED_GREEN);

    control_unlock();
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

    enum axis_mode pitch = vals->pitch.override ? vals->pitch.mode : control.pid_pitch.mode;
    enum axis_mode roll = vals->roll.override ? vals->roll.mode : control.pid_roll.mode;
    enum axis_mode yaw = vals->yaw.override ? vals->yaw.mode : control.pid_yaw.mode;
    control_set_axis_modes(pitch, roll, yaw);

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

result_t control_set_axis_modes(enum axis_mode pitch, enum axis_mode roll, enum axis_mode yaw)
{
    control_lock();

    control.pid_pitch.mode = pitch;
    control.pid_roll.mode = roll;
    control.pid_yaw.mode = yaw;

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
    res = limit_control(CONTROL_THROTTLE, cmd->throttle.absolute, false, true, cmd->throttle.value, &out->throttle);
    if (res != RES_OK)
        return res;

    res = calc_axis_control(&cmd->pitch, &out->pitch, &control.pid_pitch, CONTROL_PITCH, CONTROL_PITCH_RATE);
    if (res != RES_OK)
        return res;
    res = calc_axis_control(&cmd->roll, &out->roll, &control.pid_roll, CONTROL_ROLL, CONTROL_ROLL_RATE);
    if (res != RES_OK)
        return res;
    res = calc_axis_control(&cmd->yaw, &out->yaw, &control.pid_yaw, CONTROL_YAW, CONTROL_YAW_RATE);
    if (res != RES_OK)
        return res;

    return RES_OK;
}

//-----------------------------------------------------------------

static result_t calc_axis_control(const struct control_axis *cmd, struct control_axis_val *out,
        const struct pid_axis *pid, enum control_type type_angle, enum control_type type_rate)
{
    if (!cmd || !out || !pid)
        return RES_ERR_BAD_PARAM;

    out->override = cmd->mode.override;
    if (out->override)
        out->mode = (enum axis_mode)cmd->mode.mode;
    else
        out->mode = pid->mode;

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

    return limit_control(type, cmd->value.absolute, true, true, cmd->value.value, &out->value);
}

//-----------------------------------------------------------------

static result_t limit_control(enum control_type type, bool absolute, bool bipolar, bool dead, float val, float *out)
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

    if (dead && (fabsf(val) < limit.dead_zone))
        val = 0.0f;

    *out = val;
    return RES_OK;
}

//-----------------------------------------------------------------

static void process_arming(bool arm)
{
    if (control.armed && !arm)
        control_disarm();
    else if (!control.armed && arm)
        control_arm();
}

//-----------------------------------------------------------------

static void rcp_cb_control(struct rcp_msg *msg)
{
    control_lock();

    struct cmd_control *cmd = &msg->control;

    process_arming((bool)cmd->flags.armed);

    struct control_vals vals;
    if (calc_control(cmd, &vals) == RES_OK)
        control_set_vals(&vals);

    control_unlock();
}

//-----------------------------------------------------------------

static void rcp_cb_pid_set(struct rcp_msg *msg)
{
    struct pid *pid = control_get_pid((enum control_type)msg->pid.type);
    if (!pid)
        return;

    control_lock();

    pid->params.kp = msg->pid.kp;
    pid->params.kd = msg->pid.kd;
    pid->params.ki = msg->pid.ki;
    pid->params.kt = msg->pid.kt;
    pid->params.out_min = msg->pid.out_min;
    pid->params.out_max = msg->pid.out_max;

    pid_reset(pid);

    control_unlock();

    buzzer_seq_lib_play(BUZZER_SEQ_CONFIRM, BUZZER_MODE_QUEUE);
}

//-----------------------------------------------------------------

static void rcp_cb_pid_get(struct rcp_msg *msg)
{
    struct rcp_msg resp;
    resp.cmd    = RCP_CMD_PID;
    resp.query  = RCP_CMD_OK;

    struct pid *pid = control_get_pid((enum control_type)msg->pid.type);
    if (!pid)
        return;

    resp.pid.type = msg->pid.type;
    resp.pid.kp = pid->params.kp;
    resp.pid.ki = pid->params.ki;
    resp.pid.kd = pid->params.kd;
    resp.pid.kt = pid->params.kt;
    resp.pid.out_min = pid->params.out_min;
    resp.pid.out_max = pid->params.out_max;

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

    buzzer_seq_lib_play(BUZZER_SEQ_CONFIRM, BUZZER_MODE_QUEUE);
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

//-----------------------------------------------------------------

static void confirm_mode_set(enum axis_mode mode)
{
    enum buzzer_seq seq;
    switch (mode)
    {
    case AXIS_MODE_DISABLED:
        seq = BUZZER_SEQ_AXIS_DISABLED;
        break;
    case AXIS_MODE_ANGLE:
        seq = BUZZER_SEQ_AXIS_ANGLE;
        break;
    case AXIS_MODE_RATE:
        seq = BUZZER_SEQ_AXIS_RATE;
        break;
    default:
        seq = BUZZER_SEQ_ERROR;
        break;
    }
    buzzer_seq_lib_play(seq, BUZZER_MODE_QUEUE);
}

//-----------------------------------------------------------------

static void rcp_cb_axis_modes_set(struct rcp_msg *msg)
{
    struct cmd_axis_modes *cmd = &msg->axis_modes;
    control_set_axis_modes((enum axis_mode)cmd->pitch,
            (enum axis_mode)cmd->roll, (enum axis_mode)cmd->yaw);

    confirm_mode_set((enum axis_mode)cmd->pitch);
    confirm_mode_set((enum axis_mode)cmd->roll);
    confirm_mode_set((enum axis_mode)cmd->yaw);
}

//-----------------------------------------------------------------

static void rcp_cb_axis_modes_get(struct rcp_msg *msg)
{
    struct rcp_msg resp;
    resp.cmd    = RCP_CMD_AXIS_MODES;
    resp.query  = RCP_CMD_OK;

    control_lock();

    resp.axis_modes.pitch = (uint8_t)control.pid_pitch.mode;
    resp.axis_modes.roll = (uint8_t)control.pid_roll.mode;
    resp.axis_modes.yaw = (uint8_t)control.pid_yaw.mode;

    control_unlock();

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void rcp_cb_motors(struct rcp_msg *msg)
{
    control_lock();

    struct cmd_motors *cmd = &msg->motors;

    control.override_motors = (bool)cmd->override;

    if (control.override_motors)
    {
        process_arming((bool)cmd->armed);

        control.override_vals.m1 = cmd->m1;
        control.override_vals.m2 = cmd->m2;
        control.override_vals.m3 = cmd->m3;
        control.override_vals.m4 = cmd->m4;
    }

    control_unlock();
}
