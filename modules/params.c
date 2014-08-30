/*
 * params.c
 *
 *  Created on: 14 cze 2014
 *      Author: Korzo
 */

#include "params.h"

#include <drivers/eeprom.h>
#include <modules/rcp.h>
#include <utils/buzzer_seq.h>
#include <utils/delay.h>
#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>

//-----------------------------------------------------------------

#define PARAM_EEPROM_ADDR       0x0000

#define PARAM_DEF(_group, _name, _type, _size, _count, _ptr)    \
        { .group = _group, .name = _name, .type = _type, .size = _size, .count = _count, .ptr = _ptr }

#define PARAM_DEF_FLOAT(_group, _name, _count, _ptr)    \
        PARAM_DEF(_group, _name, PARAM_TYPE_FLOAT, sizeof(float), _count, _ptr)

#define PARAM_PID_BATCH(_group, _pid)                       \
        PARAM_DEF_FLOAT(_group, "kp", 1, &_pid.kp),         \
        PARAM_DEF_FLOAT(_group, "ki", 1, &_pid.ki),         \
        PARAM_DEF_FLOAT(_group, "kd", 1, &_pid.kd),         \
        PARAM_DEF_FLOAT(_group, "kt", 1, &_pid.kt),         \
        PARAM_DEF_FLOAT(_group, "out_max", 1, &_pid.kt),    \
        PARAM_DEF_FLOAT(_group, "out_min", 1, &_pid.kt)

#define PID_DEFAULTS(_kp, _ki, _kd, _kt, _out_max, _out_min, _der)    \
        (struct pid_params){                  \
            .kp = _kp, .ki = _ki, .kd = _kd,  \
            .out_max = _out_max,              \
            .out_min = _out_min,              \
            .deriv = _der                     \
        }

#define PARAM_LIMIT(_group, _lim)                                   \
        PARAM_DEF_FLOAT(_group, "limit", 1, &_lim.limit),           \
        PARAM_DEF_FLOAT(_group, "dead_zone", 1, &_lim.dead_zone)

#define PARAM_LIMIT_AXES(_pre, _suf, _lim)          \
        PARAM_LIMIT(_pre"pitch"_suf, _lim.pitch),   \
        PARAM_LIMIT(_pre"roll"_suf, _lim.roll),     \
        PARAM_LIMIT(_pre"yaw"_suf, _lim.yaw)

#define LIMIT_DEFAULTS(_lim, _dead) \
        (struct control_limit){     \
            .limit      = _lim,     \
            .dead_zone  = _dead     \
        }

#define PARAM_AXIS_MODE(_name, _ptr)    \
        PARAM_DEF("axis_mode", _name, PARAM_TYPE_UINT32, sizeof(enum axis_mode), 1, _ptr)


//-----------------------------------------------------------------

struct params_obj
{
    xSemaphoreHandle mutex;

    struct pid_params pid_pitch;
    struct pid_params pid_roll;
    struct pid_params pid_yaw;
    struct pid_params pid_pitch_rate;
    struct pid_params pid_roll_rate;
    struct pid_params pid_yaw_rate;

    float   mag_calib_scale[9];
    float   mag_calib_offset[3];

    struct vec3  triad_ref_acc;
    struct vec3  triad_ref_mag;

    float   madgwick_beta;

    struct control_limit        limit_throttle;
    struct control_limit_axes   limit_angles;
    struct control_limit_axes   limit_rates;

    enum axis_mode mode_pitch;
    enum axis_mode mode_roll;
    enum axis_mode mode_yaw;
};

static struct params_obj params;

//-----------------------------------------------------------------

const struct param_info infos[] = {
    PARAM_PID_BATCH("pid_pitch", params.pid_pitch),
    PARAM_PID_BATCH("pid_roll", params.pid_roll),
    PARAM_PID_BATCH("pid_yaw", params.pid_yaw),
    PARAM_PID_BATCH("pid_pitch_rate", params.pid_pitch_rate),
    PARAM_PID_BATCH("pid_roll_rate", params.pid_roll_rate),
    PARAM_PID_BATCH("pid_yaw_rate", params.pid_yaw_rate),

    PARAM_DEF_FLOAT("mag_calib", "scale", 9, params.mag_calib_scale),
    PARAM_DEF_FLOAT("mag_calib", "offset", 3, params.mag_calib_offset),

    PARAM_DEF_FLOAT("triad", "ref_acc", 3, &params.triad_ref_acc),
    PARAM_DEF_FLOAT("triad", "ref_mag", 3, &params.triad_ref_mag),

    PARAM_DEF_FLOAT("madgwick", "beta", 1, &params.madgwick_beta),

    PARAM_LIMIT("lim_throttle", params.limit_throttle),
    PARAM_LIMIT_AXES("lim_", "", params.limit_angles),
    PARAM_LIMIT_AXES("lim_", "_rate", params.limit_rates),

    PARAM_AXIS_MODE("pitch", &params.mode_pitch),
    PARAM_AXIS_MODE("roll", &params.mode_roll),
    PARAM_AXIS_MODE("yaw", &params.mode_yaw)
};

//-----------------------------------------------------------------

static uint16_t params_count_bytes(void);

static void params_lock(void);
static void params_unlock(void);

static const struct param_info* params_get_info(uint8_t id);

static void rcp_cb_list(struct rcp_msg *msg);
static void rcp_cb_info(struct rcp_msg *msg);
static void rcp_cb_get(struct rcp_msg *msg);
static void rcp_cb_set(struct rcp_msg *msg);
static void rcp_cb_save(struct rcp_msg *msg);

static result_t params_copy(void *out, const void *src, size_t size);
static result_t params_get_pid(struct pid_params *out, struct pid_params *src);

static struct control_limit* get_limit(enum control_type type);

//-----------------------------------------------------------------

result_t params_init(void)
{
    params.mutex = xSemaphoreCreateRecursiveMutex();

    rcp_register_callback(RCP_CMD_PARAM_LIST, rcp_cb_list, true);
    rcp_register_callback(RCP_CMD_PARAM_INFO, rcp_cb_info, true);
    rcp_register_callback(RCP_CMD_PARAM_GET,  rcp_cb_get,  true);
    rcp_register_callback(RCP_CMD_PARAM_SET,  rcp_cb_set,  false);
    rcp_register_callback(RCP_CMD_PARAM_SAVE, rcp_cb_save, false);

    params_load_defaults();

    return RES_OK;
}

//-----------------------------------------------------------------

static void params_lock(void)
{
    xSemaphoreTakeRecursive(params.mutex, portMAX_DELAY);
}

//-----------------------------------------------------------------

static void params_unlock(void)
{
    xSemaphoreGiveRecursive(params.mutex);
}

//-----------------------------------------------------------------

void params_load_defaults(void)
{
    params_lock();

    params.pid_pitch        = PID_DEFAULTS(1.0f, 0.0f, 0.0f, 0.5f, 1000.0f, 0.0f, PID_DERIV_ON_ERROR);
    params.pid_roll         = PID_DEFAULTS(1.0f, 0.0f, 0.0f, 0.5f, 20.0f, 0.0f, PID_DERIV_ON_ERROR);
    params.pid_yaw          = PID_DEFAULTS(1.0f, 0.0f, 0.0f, 0.5f, 20.0f, 0.0f, PID_DERIV_ON_ERROR);
    params.pid_pitch_rate   = PID_DEFAULTS(1.0f, 0.1f, 0.0f, 0.5f, 1000.0f, -1000.0f, PID_DERIV_ON_ERROR);
    params.pid_roll_rate    = PID_DEFAULTS(1.0f, 0.0f, 0.0f, 0.5f, 1000.0f, 0.0f, PID_DERIV_ON_ERROR);
    params.pid_yaw_rate     = PID_DEFAULTS(1.0f, 0.0f, 0.0f, 0.5f, 1000.0f, 0.0f, PID_DERIV_ON_ERROR);

    const float def_mag_calib_scale[] = {
        0.70892, 0.00171642, 0.0316637,
        0.00171642, 0.770402, 0.0256191,
        0.0316637, 0.0256191, 0.993655
    };
    memcpy(params.mag_calib_scale, def_mag_calib_scale, sizeof(def_mag_calib_scale));

    const float def_mag_calib_offset[] = {
        52.541, -147.339, -98.677
    };
    memcpy(params.mag_calib_offset, def_mag_calib_offset, sizeof(def_mag_calib_offset));

    params.triad_ref_acc = VEC3_NEW(-0.0156, 0.0391, 0.9375);
    params.triad_ref_mag = VEC3_NEW(131.6290, 11.9624, -343.7395);

    params.madgwick_beta = 0.4f;

    params.limit_throttle       = LIMIT_DEFAULTS(1000.0f, 50.0f);
    params.limit_angles.pitch   = LIMIT_DEFAULTS(60.0f, 1.0f);
    params.limit_angles.roll    = LIMIT_DEFAULTS(60.0f, 1.0f);
    params.limit_angles.yaw     = LIMIT_DEFAULTS(180.0f, 1.0f);
    params.limit_rates.pitch    = LIMIT_DEFAULTS(10.0f, 0.5f);
    params.limit_rates.roll     = LIMIT_DEFAULTS(10.0f, 0.5f);
    params.limit_rates.yaw      = LIMIT_DEFAULTS(10.0f, 0.5f);

    params_unlock();
}

//-----------------------------------------------------------------

const struct param_info* params_get_info(uint8_t id)
{
    return id >= ARRAY_COUNT(infos) ? NULL : &infos[id];
}

//-----------------------------------------------------------------

result_t params_eeprom_save(void)
{
    params_lock();

    uint16_t addr = PARAM_EEPROM_ADDR;

    // write param count
    uint8_t count = ARRAY_COUNT(infos);
    if (eeprom_write_byte(addr++, count) != RES_OK)
    {
        params_unlock();
        return RES_ERR_IO;
    }
    while (!eeprom_is_ready());

    // write bytes count
    uint16_t bytes = params_count_bytes();
    uint8_t *ptr = (uint8_t*)&bytes;
    if (eeprom_write(addr++, ptr++, 1) != RES_OK)
    {
        params_unlock();
        return RES_ERR_IO;
    }
    while (!eeprom_is_ready());
    if (eeprom_write(addr++, ptr, 1) != RES_OK)
    {
        params_unlock();
        return RES_ERR_IO;
    }
    while (!eeprom_is_ready());

    uint8_t i;
    for (i = 0; i < count; i++)
    {
        const struct param_info *param = &infos[i];

        // write param bytes count
        bytes = param->count * param->size;
        ptr = (uint8_t*)&bytes;
        if (eeprom_write(addr++, ptr++, 1) != RES_OK)
        {
            params_unlock();
            return RES_ERR_IO;
        }
        while (!eeprom_is_ready());
        if (eeprom_write(addr++, ptr, 1) != RES_OK)
        {
            params_unlock();
            return RES_ERR_IO;
        }
        while (!eeprom_is_ready());

        // write param data
        ptr = (uint8_t*)param->ptr;
        uint16_t j;
        for (j = 0; j < bytes; j++)
        {
            if (eeprom_write(addr++, ptr++, 1) != RES_OK)
            {
                params_unlock();
                return RES_ERR_IO;
            }
            while (!eeprom_is_ready());
        }
    }

    params_unlock();
    return RES_OK;
}

//-----------------------------------------------------------------

result_t params_eeprom_load(void)
{
    params_lock();

    uint16_t addr = PARAM_EEPROM_ADDR;

    uint8_t count;
    if (eeprom_read_byte(addr++, &count) != RES_OK)
    {
        params_unlock();
        return RES_ERR_IO;
    }
    // check for param count match
    if (count != ARRAY_COUNT(infos))
    {
        params_unlock();
        return RES_ERR_FATAL;
    }

    uint16_t bytes;
    uint8_t *ptr = (uint8_t*)&bytes;
    if (eeprom_read(addr, ptr, 2) != RES_OK)
    {
        params_unlock();
        return RES_ERR_IO;
    }
    // check for bytes count match
    if (bytes != params_count_bytes())
    {
        params_unlock();
        return RES_ERR_FATAL;
    }
    addr += 2;

    uint8_t i;
    for (i = 0; i < count; i++)
    {
        const struct param_info *param = &infos[i];

        ptr = (uint8_t*)&bytes;
        if (eeprom_read(addr, ptr, 2) != RES_OK)
        {
            params_unlock();
            return RES_ERR_IO;
        }
        // check for param bytes count match
        if (bytes != param->count * param->size)
        {
            params_unlock();
            return RES_ERR_FATAL;
        }
        addr += 2;

        ptr = (uint8_t*)param->ptr;
        // read param data
        if (eeprom_read(addr, ptr, bytes) != RES_OK)
        {
            params_unlock();
            return RES_ERR_IO;
        }
        addr += bytes;
    }

    params_unlock();
    return RES_OK;
}

//-----------------------------------------------------------------

static uint16_t params_count_bytes(void)
{
    uint16_t num = 0;

    int i;
    for (i = 0; i < ARRAY_COUNT(infos); i++)
    {
        const struct param_info *param = &infos[i];
        num += param->count * param->size;
    }

    return num;
}

//-----------------------------------------------------------------

static void rcp_cb_list(struct rcp_msg *msg)
{
    struct rcp_msg resp;
    resp.cmd    = RCP_CMD_PARAM_LIST;
    resp.query  = RCP_CMD_OK;

    resp.param_list.count = ARRAY_COUNT(infos);

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void rcp_cb_info(struct rcp_msg *msg)
{
    struct rcp_msg resp;
    resp.cmd = RCP_CMD_PARAM_INFO;

    uint8_t id = msg->param_info.id;

    const struct param_info *param = params_get_info(id);
    if (!param)
    {
        resp.query = RCP_CMD_ERROR;
    }
    else
    {
        resp.query = RCP_CMD_OK;

        strncpy(resp.param_info.group, param->group, PARAM_MAX_GROUP_LEN);
        strncpy(resp.param_info.name, param->name, PARAM_MAX_NAME_LEN);
        resp.param_info.id    = id;
        resp.param_info.type  = (uint8_t)param->type;
        resp.param_info.size  = param->size;
        resp.param_info.count = param->count;
    }

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void rcp_cb_get(struct rcp_msg *msg)
{
    struct cmd_param_data *args = &msg->param_data;

    struct rcp_msg resp;
    resp.cmd = RCP_CMD_PARAM_GET;

    const struct param_info *param = params_get_info(args->id);
    if (!param || args->offset >= param->count)
    {
        resp.query = RCP_CMD_ERROR;
    }
    else
    {
        resp.query = RCP_CMD_OK;

        uint8_t avail = ARRAY_COUNT(resp.param_data.data) / param->size;
        uint8_t rem = param->count - args->offset;
        uint8_t num = min(avail, rem);
        num = min(msg->param_data.count, num);
        uint8_t off = args->offset * param->size;

        memset(&resp.param_data, 0, sizeof(resp.param_data));
        params_lock();
        memcpy(resp.param_data.data, (uint8_t*)param->ptr + off, num * param->size);
        params_unlock();

        resp.param_data.id     = args->id;
        resp.param_data.offset = args->offset;
        resp.param_data.count  = num;
    }

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void rcp_cb_set(struct rcp_msg *msg)
{
    struct cmd_param_data *args = &msg->param_data;

    const struct param_info *param = params_get_info(args->id);
    if (!param || args->offset >= param->count)
        return;

    uint8_t avail = ARRAY_COUNT(args->data) / param->size;
    uint8_t num = min(avail, param->count);
    num = min(args->count, num);
    uint8_t off = args->offset * param->size;

    params_lock();
    memcpy((uint8_t*)param->ptr + off, args->data, num * param->size);
    params_unlock();

    buzzer_seq_lib_play(BUZZER_SEQ_CONFIRM);
}

//-----------------------------------------------------------------

static void rcp_cb_save(struct rcp_msg *msg)
{
    (void)msg;
    if (params_eeprom_save() == RES_OK)
        buzzer_seq_lib_play(BUZZER_SEQ_CONFIRM);
}

//-----------------------------------------------------------------

static result_t params_copy(void *out, const void *src, size_t size)
{
    if (!out || !src)
        return RES_ERR_BAD_PARAM;
    params_lock();
    memcpy(out, src, size);
    params_unlock();
    return RES_OK;
}

//-----------------------------------------------------------------

static result_t params_get_pid(struct pid_params *out, struct pid_params *pid)
{
    return params_copy(out, pid, sizeof(struct pid_params));
}

//-----------------------------------------------------------------

result_t params_get_pid_pitch(struct pid_params *out)
{
    return params_get_pid(out, &params.pid_pitch);
}

//-----------------------------------------------------------------

result_t params_get_pid_roll(struct pid_params *out)
{
    return params_get_pid(out, &params.pid_roll);
}

//-----------------------------------------------------------------

result_t params_get_pid_yaw(struct pid_params *out)
{
    return params_get_pid(out, &params.pid_yaw);
}

//-----------------------------------------------------------------

result_t params_get_pid_pitch_rate(struct pid_params *out)
{
    return params_get_pid(out, &params.pid_pitch_rate);
}

//-----------------------------------------------------------------

result_t params_get_pid_roll_rate(struct pid_params *out)
{
    return params_get_pid(out, &params.pid_roll_rate);
}

//-----------------------------------------------------------------

result_t params_get_pid_yaw_rate(struct pid_params *out)
{
    return params_get_pid(out, &params.pid_yaw_rate);
}

//-----------------------------------------------------------------

result_t params_get_mag_calib_scale(float *out)
{
    return params_copy(out, params.mag_calib_scale, sizeof(params.mag_calib_scale));
}

//-----------------------------------------------------------------

result_t params_get_mag_calib_offset(float *out)
{
    return params_copy(out, params.mag_calib_offset, sizeof(params.mag_calib_offset));
}

//-----------------------------------------------------------------

result_t params_get_triad_ref_acc(struct vec3 *out)
{
    return params_copy(out, &params.triad_ref_acc, sizeof(struct vec3));
}

//-----------------------------------------------------------------

result_t params_get_triad_ref_mag(struct vec3 *out)
{
    return params_copy(out, &params.triad_ref_mag, sizeof(struct vec3));
}

//-----------------------------------------------------------------

result_t params_get_madgwick_beta(float *out)
{
    return params_copy(out, &params.madgwick_beta, sizeof(float));
}

//-----------------------------------------------------------------

static struct control_limit* get_limit(enum control_type type)
{
    switch (type)
    {
    case CONTROL_THROTTLE:
        return &params.limit_throttle;
    case CONTROL_PITCH:
        return &params.limit_angles.pitch;
    case CONTROL_ROLL:
        return &params.limit_angles.roll;
    case CONTROL_YAW:
        return &params.limit_angles.yaw;
    case CONTROL_PITCH_RATE:
        return &params.limit_rates.pitch;
    case CONTROL_ROLL_RATE:
        return &params.limit_rates.roll;
    case CONTROL_YAW_RATE:
        return &params.limit_rates.yaw;
    default:
        return NULL;
    }
}

//-----------------------------------------------------------------

result_t params_get_limit(enum control_type type, struct control_limit *out)
{
    struct control_limit *par = get_limit(type);
    return params_copy(out, par, sizeof(struct control_limit));
}

//-----------------------------------------------------------------

result_t params_set_limit(enum control_type type, const struct control_limit *limit)
{
    struct control_limit *par = get_limit(type);
    return params_copy(par, limit, sizeof(struct control_limit));
}

//-----------------------------------------------------------------

result_t params_get_pitch_mode(enum axis_mode *out)
{
    return params_copy(out, &params.mode_pitch, sizeof(enum axis_mode));
}

//-----------------------------------------------------------------

result_t params_get_roll_mode(enum axis_mode *out)
{
    return params_copy(out, &params.mode_roll, sizeof(enum axis_mode));
}

//-----------------------------------------------------------------

result_t params_get_yaw_mode(enum axis_mode *out)
{
    return params_copy(out, &params.mode_yaw, sizeof(enum axis_mode));
}
