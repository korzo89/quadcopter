/*
 * daq.c
 *
 *  Created on: 26 cze 2014
 *      Author: Korzo
 */

#include "daq.h"

#include <modules/rcp.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <stdlib.h>
#include <string.h>

//-----------------------------------------------------------------

#define DAQ_VALUES_MAX      50

#define DAQ_GET_VALS_MAX    6

#define DAQ_CONVERT(p, t)   ( (float)*((t*)p) )

//-----------------------------------------------------------------

static struct daq_value *values[DAQ_VALUES_MAX];
static uint8_t values_num = 0;

static xSemaphoreHandle mutex;

//-----------------------------------------------------------------

static void rcp_daq_list_cb(rcp_message_t *msg);
static void rcp_daq_info_cb(rcp_message_t *msg);
static void rcp_daq_get_cb(rcp_message_t *msg);

//-----------------------------------------------------------------

result_t daq_init(void)
{
    mutex = xSemaphoreCreateMutex();

    rcp_register_callback(RCP_CMD_DAQ_LIST, rcp_daq_list_cb, true);
    rcp_register_callback(RCP_CMD_DAQ_INFO, rcp_daq_info_cb, true);
    rcp_register_callback(RCP_CMD_DAQ_GET, rcp_daq_get_cb, true);

    return RES_OK;
}

//-----------------------------------------------------------------

result_t daq_register_value(const char *name, const char *unit, void *ptr, enum daq_type type)
{
    if (!xSemaphoreTake(mutex, portMAX_DELAY))
        return RES_ERR_BLOCKED;

    if (!ptr || !name || !unit)
        return RES_ERR_BAD_PARAM;
    if (values_num >= DAQ_VALUES_MAX)
        return RES_ERR_FATAL;

    struct daq_value *new_val = malloc(sizeof(struct daq_value));
    if (!new_val)
        return RES_ERR_FATAL;

    new_val->name = name;
    new_val->unit = unit;
    new_val->type = type;
    new_val->ptr = ptr;

    values[values_num++] = new_val;

    xSemaphoreGive(mutex);
    return RES_OK;
}

//-----------------------------------------------------------------

struct daq_value* daq_get_info(uint8_t id)
{
    return id >= values_num ? NULL : values[id];
}

//-----------------------------------------------------------------

result_t daq_get_value(uint8_t id, float *out)
{
    if (!out)
        return RES_ERR_BAD_PARAM;

    struct daq_value *val = daq_get_info(id);
    if (!val)
        return RES_ERR_FATAL;

    switch (val->type)
    {
    case DAQ_TYPE_FLOAT:
        *out = DAQ_CONVERT(val->ptr, float);
        break;
    case DAQ_TYPE_INT:
        *out = DAQ_CONVERT(val->ptr, int);
        break;
    case DAQ_TYPE_UINT8:
        *out = DAQ_CONVERT(val->ptr, uint8_t);
        break;
    case DAQ_TYPE_UINT16:
        *out = DAQ_CONVERT(val->ptr, uint16_t);
        break;
    case DAQ_TYPE_UINT32:
        *out = DAQ_CONVERT(val->ptr, uint32_t);
        break;
    case DAQ_TYPE_INT8:
        *out = DAQ_CONVERT(val->ptr, int8_t);
        break;
    case DAQ_TYPE_INT16:
        *out = DAQ_CONVERT(val->ptr, int16_t);
        break;
    case DAQ_TYPE_INT32:
        *out = DAQ_CONVERT(val->ptr, int32_t);
        break;
    }

    return RES_OK;
}

//-----------------------------------------------------------------

static void rcp_daq_list_cb(rcp_message_t *msg)
{
    rcp_message_t resp;
    resp.packet.cmd = RCP_CMD_DAQ_LIST;
    resp.packet.query = RCP_CMD_OK;

    uint8_t *count = (uint8_t*)resp.packet.data;
    *count = values_num;

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

typedef struct PACK_STRUCT
{
    uint8_t id;
    char    name[DAQ_NAME_MAX_LEN];
    char    unit[DAQ_UNIT_MAX_LEN];
} daq_info_data_t;

static void rcp_daq_info_cb(rcp_message_t *msg)
{
    rcp_message_t resp;
    resp.packet.cmd = RCP_CMD_DAQ_INFO;

    uint8_t id = *((uint8_t*)msg->packet.data);

    const struct daq_value *val = daq_get_info(id);
    if (!val)
    {
        resp.packet.query = RCP_CMD_ERROR;
    }
    else
    {
        resp.packet.query = RCP_CMD_OK;

        daq_info_data_t *data = (daq_info_data_t*)resp.packet.data;
        strncpy(data->name, val->name, DAQ_NAME_MAX_LEN);
        strncpy(data->unit, val->unit, DAQ_UNIT_MAX_LEN);
        data->id = id;
    }

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

typedef struct PACK_STRUCT
{
    uint32_t    time;
    uint8_t     num;
    float       values[DAQ_GET_VALS_MAX];
} daq_get_data_t;

static void rcp_daq_get_cb(rcp_message_t *msg)
{
    rcp_message_t resp;
    resp.packet.cmd = RCP_CMD_DAQ_INFO;
    resp.packet.query = RCP_CMD_OK;

    uint8_t *args = (uint8_t*)msg->packet.data;
    uint8_t num = *args;
    if (num > DAQ_GET_VALS_MAX)
        num = DAQ_GET_VALS_MAX;
    args++;

    daq_get_data_t *data = (daq_get_data_t*)resp.packet.data;
    data->time = (uint32_t)xTaskGetTickCount();
    data->num = num;

    uint8_t i;
    for (i = 0; i < num; i++)
    {
        if (daq_get_value(args[i], &data->values[i]) != RES_OK)
        {
            resp.packet.query = RCP_CMD_ERROR;
            break;
        }
    }

    rcp_send_message(&resp);
}
