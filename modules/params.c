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
#include <utils/pid.h>
#include <utils/delay.h>
#include <string.h>

//-----------------------------------------------------------------

#define PARAM_EXTERN(x)         extern x

#define PARAM_MAX_GROUP_LEN     15
#define PARAM_MAX_NAME_LEN      11

#define PARAM_EEPROM_ADDR       0x0000

#define PARAM_PID_BATCH(n, p)   \
    { n, "Kp", PARAM_TYPE_FLOAT, sizeof(float), 1, &p.kp }, \
    { n, "Ki", PARAM_TYPE_FLOAT, sizeof(float), 1, &p.ki }, \
    { n, "Kd", PARAM_TYPE_FLOAT, sizeof(float), 1, &p.kd }, \
    { n, "Kt", PARAM_TYPE_FLOAT, sizeof(float), 1, &p.kt }

//-----------------------------------------------------------------

PARAM_EXTERN(pid_t pid_pitch);
PARAM_EXTERN(pid_t pid_roll);
PARAM_EXTERN(pid_t pid_yaw);
PARAM_EXTERN(pid_t pid_pitch_rate);
PARAM_EXTERN(pid_t pid_roll_rate);
PARAM_EXTERN(pid_t pid_yaw_rate);
PARAM_EXTERN(float mag_calib_scale[]);
PARAM_EXTERN(float mag_calib_offset[]);

const param_info_t PARAMS[] = {
    PARAM_PID_BATCH("PID_pitch", pid_pitch),
    PARAM_PID_BATCH("PID_roll", pid_roll),
    PARAM_PID_BATCH("PID_yaw", pid_yaw),
    PARAM_PID_BATCH("PID_pitch_rate", pid_pitch_rate),
    PARAM_PID_BATCH("PID_roll_rate", pid_roll_rate),
    PARAM_PID_BATCH("PID_yaw_rate", pid_yaw_rate),
    { "Mag_calib",  "scale",    PARAM_TYPE_FLOAT, sizeof(float), 9, mag_calib_scale },
    { "Mag_calib",  "offset",   PARAM_TYPE_FLOAT, sizeof(float), 3, mag_calib_offset }
};

//-----------------------------------------------------------------

static uint16_t params_count_bytes(void);

static void rcp_cb_list(rcp_message_t *msg);
static void rcp_cb_info(rcp_message_t *msg);
static void rcp_cb_get(rcp_message_t *msg);
static void rcp_cb_set(rcp_message_t *msg);
static void rcp_cb_save(rcp_message_t *msg);

//-----------------------------------------------------------------

result_t params_init(void)
{
    rcp_register_callback(RCP_CMD_PARAM_LIST, rcp_cb_list, true);
    rcp_register_callback(RCP_CMD_PARAM_INFO, rcp_cb_info, true);
    rcp_register_callback(RCP_CMD_PARAM_GET,  rcp_cb_get,  true);
    rcp_register_callback(RCP_CMD_PARAM_SET,  rcp_cb_set,  false);
    rcp_register_callback(RCP_CMD_PARAM_SET,  rcp_cb_set,  false);
    rcp_register_callback(RCP_CMD_PARAM_SAVE,  rcp_cb_save,  false);

    return RES_OK;
}

//-----------------------------------------------------------------

const param_info_t* params_get_info(uint8_t id)
{
    return id >= ARRAY_COUNT(PARAMS) ? NULL : &PARAMS[id];
}

//-----------------------------------------------------------------

result_t params_eeprom_save(void)
{
//    portENTER_CRITICAL();

    uint16_t addr = PARAM_EEPROM_ADDR;

    // write param count
    uint8_t count = ARRAY_COUNT(PARAMS);
    if (eeprom_write_byte(addr++, count) != RES_OK)
        return RES_ERR_IO;
    while (!eeprom_is_ready());

    // write bytes count
    uint16_t bytes = params_count_bytes();
    uint8_t *ptr = (uint8_t*)&bytes;
    if (eeprom_write(addr++, ptr++, 1) != RES_OK)
        return RES_ERR_IO;
    while (!eeprom_is_ready());
    if (eeprom_write(addr++, ptr, 1) != RES_OK)
        return RES_ERR_IO;
    while (!eeprom_is_ready());

    uint8_t i;
    for (i = 0; i < count; i++)
    {
        const param_info_t *param = &PARAMS[i];

        // write param bytes count
        bytes = param->count * param->size;
        ptr = (uint8_t*)&bytes;
        if (eeprom_write(addr++, ptr++, 1) != RES_OK)
            return RES_ERR_IO;
        while (!eeprom_is_ready());
        if (eeprom_write(addr++, ptr, 1) != RES_OK)
            return RES_ERR_IO;
        while (!eeprom_is_ready());

        // write param data
        ptr = (uint8_t*)param->ptr;
        uint16_t j;
        for (j = 0; j < bytes; j++)
        {
            if (eeprom_write(addr++, ptr++, 1) != RES_OK)
                return RES_ERR_IO;
            while (!eeprom_is_ready());
        }
    }

//    portEXIT_CRITICAL();

    return RES_OK;
}

//-----------------------------------------------------------------

result_t params_eeprom_load(void)
{
    uint16_t addr = PARAM_EEPROM_ADDR;

    uint8_t count;
    if (eeprom_read_byte(addr++, &count) != RES_OK)
        return RES_ERR_IO;
    // check for param count match
    if (count != ARRAY_COUNT(PARAMS))
        return RES_ERR_FATAL;

    uint16_t bytes;
    uint8_t *ptr = (uint8_t*)&bytes;
    if (eeprom_read(addr, ptr, 2) != RES_OK)
        return RES_ERR_IO;
    // check for bytes count match
    if (bytes != params_count_bytes())
        return RES_ERR_FATAL;
    addr += 2;

    uint8_t i;
    for (i = 0; i < count; i++)
    {
        const param_info_t *param = &PARAMS[i];

        ptr = (uint8_t*)&bytes;
        if (eeprom_read(addr, ptr, 2) != RES_OK)
            return RES_ERR_IO;
        // check for param bytes count match
        if (bytes != param->count * param->size)
            return RES_ERR_FATAL;
        addr += 2;

        ptr = (uint8_t*)param->ptr;
        // read param data
        if (eeprom_read(addr, ptr, bytes) != RES_OK)
            return RES_ERR_IO;
        addr += bytes;
    }

    return RES_OK;
}

//-----------------------------------------------------------------

static uint16_t params_count_bytes(void)
{
    uint16_t num = 0;

    int i;
    for (i = 0; i < ARRAY_COUNT(PARAMS); i++)
    {
        const param_info_t *param = &PARAMS[i];
        num += param->count * param->size;
    }

    return num;
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

        data->id     = args->id;
        data->offset = args->offset;
        data->count  = num;
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

//-----------------------------------------------------------------

static void rcp_cb_save(rcp_message_t *msg)
{
    if (params_eeprom_save() == RES_OK)
        buzzer_seq_lib_play(BUZZER_SEQ_CONFIRM);
}
