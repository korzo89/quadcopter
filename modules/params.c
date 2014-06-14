/*
 * params.c
 *
 *  Created on: 14 cze 2014
 *      Author: Korzo
 */

#include "params.h"

#include <modules/rcp.h>
#include <utils/buzzer_seq.h>
#include <utils/pid.h>
#include <string.h>

//-----------------------------------------------------------------

#define PARAM_EXTERN(x)         extern x

#define PARAM_MAX_GROUP_LEN     15
#define PARAM_MAX_NAME_LEN      11

//-----------------------------------------------------------------

PARAM_EXTERN(pid_t pid_pitch);
PARAM_EXTERN(pid_t pid_roll);
PARAM_EXTERN(float mag_calib_scale[]);
PARAM_EXTERN(float mag_calib_offset[]);

const param_info_t PARAMS[] = {
    { "PID_pitch",  "Kp",       PARAM_TYPE_FLOAT, sizeof(float), 1, &pid_pitch.kp },
    { "PID_pitch",  "Ki",       PARAM_TYPE_FLOAT, sizeof(float), 1, &pid_pitch.ki },
    { "PID_pitch",  "Kd",       PARAM_TYPE_FLOAT, sizeof(float), 1, &pid_pitch.kd },
    { "PID_roll",   "Kp",       PARAM_TYPE_FLOAT, sizeof(float), 1, &pid_roll.kp },
    { "PID_roll",   "Ki",       PARAM_TYPE_FLOAT, sizeof(float), 1, &pid_roll.ki },
    { "PID_roll",   "Kd",       PARAM_TYPE_FLOAT, sizeof(float), 1, &pid_roll.kd },
    { "Mag_calib",  "scale",    PARAM_TYPE_FLOAT, sizeof(float), 9, mag_calib_scale },
    { "Mag_calib",  "offset",   PARAM_TYPE_FLOAT, sizeof(float), 3, mag_calib_offset }
};

//-----------------------------------------------------------------

static void rcp_cb_list(rcp_message_t *msg);
static void rcp_cb_info(rcp_message_t *msg);
static void rcp_cb_get(rcp_message_t *msg);
static void rcp_cb_set(rcp_message_t *msg);

//-----------------------------------------------------------------

result_t params_init(void)
{
    rcp_register_callback(RCP_CMD_PARAM_LIST, rcp_cb_list, true);
    rcp_register_callback(RCP_CMD_PARAM_INFO, rcp_cb_info, true);
    rcp_register_callback(RCP_CMD_PARAM_GET,  rcp_cb_get,  true);
    rcp_register_callback(RCP_CMD_PARAM_SET,  rcp_cb_set,  false);

    return RES_OK;
}

//-----------------------------------------------------------------

const param_info_t* params_get_info(uint8_t id)
{
    return id >= ARRAY_COUNT(PARAMS) ? NULL : &PARAMS[id];
}

//-----------------------------------------------------------------

static void rcp_cb_list(rcp_message_t *msg)
{
    rcp_message_t resp;
    resp.packet.cmd = RCP_CMD_PARAM_LIST;
    resp.packet.query = RCP_CMD_OK;

    uint8_t *count = (uint8_t*)resp.packet.data;
    *count = ARRAY_COUNT(PARAMS);

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

typedef struct PACK_STRUCT
{
    uint8_t id;
    char    group[PARAM_MAX_GROUP_LEN];
    char    name[PARAM_MAX_NAME_LEN];
    uint8_t type;
    uint8_t size;
    uint8_t count;
} param_info_data_t;

static void rcp_cb_info(rcp_message_t *msg)
{
    rcp_message_t resp;
    resp.packet.cmd = RCP_CMD_PARAM_INFO;

    uint8_t id = *((uint8_t*)msg->packet.data);

    const param_info_t *param = params_get_info(id);
    if (!param)
    {
        resp.packet.query = RCP_CMD_ERROR;
    }
    else
    {
        resp.packet.query = RCP_CMD_OK;

        param_info_data_t *data = (param_info_data_t*)resp.packet.data;
        strncpy(data->group, param->group, PARAM_MAX_GROUP_LEN);
        strncpy(data->name, param->name, PARAM_MAX_NAME_LEN);
        data->id    = id;
        data->type  = (uint8_t)param->type;
        data->size  = param->size;
        data->count = param->count;
    }

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

typedef struct PACK_STRUCT
{
    uint8_t id;
    uint8_t offset;
    uint8_t count;
    uint8_t data[RCP_PAYLOAD_SIZE - 5];
} param_data_t;

static void rcp_cb_get(rcp_message_t *msg)
{
    rcp_message_t resp;
    resp.packet.cmd = RCP_CMD_PARAM_GET;

    param_data_t *args = (param_data_t*)msg->packet.data;

    const param_info_t *param = params_get_info(args->id);
    if (!param || args->offset >= param->count)
    {
        resp.packet.query = RCP_CMD_ERROR;
    }
    else
    {
        resp.packet.query = RCP_CMD_OK;

        param_data_t *data = (param_data_t*)resp.packet.data;

        uint8_t avail = ARRAY_COUNT(data->data) / param->size;
        uint8_t rem = param->count - args->offset;
        uint8_t num = min(avail, rem);
        num = min(args->count, num);
        uint8_t off = args->offset * param->size;

        memset(data, 0, sizeof(param_data_t));
        memcpy(data->data, (uint8_t*)param->ptr + off, num * param->size);

        data->id = args->id;
        data->offset = args->offset;
        data->count = num;
    }

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void rcp_cb_set(rcp_message_t *msg)
{
    param_data_t *args = (param_data_t*)msg->packet.data;

    const param_info_t *param = params_get_info(args->id);
    if (!param || args->offset >= param->count)
        return;

    uint8_t avail = ARRAY_COUNT(args->data) / param->size;
    uint8_t num = min(avail, param->count);
    num = min(args->count, num);
    uint8_t off = args->offset * param->size;

    memcpy((uint8_t*)param->ptr + off, args->data, num * param->size);

    buzzer_seq_lib_play(BUZZER_SEQ_CONFIRM);
}
