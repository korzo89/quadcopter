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

#define DAQ_CONVERT(p, t)   ( (float)(*((t*)p)) )

//-----------------------------------------------------------------

static struct daq_value *values[DAQ_VALUES_MAX];
static uint8_t values_num = 0;

static xSemaphoreHandle mutex;

//-----------------------------------------------------------------

static void rcp_daq_list_cb(struct rcp_msg *msg);
static void rcp_daq_info_cb(struct rcp_msg *msg);
static void rcp_daq_get_cb(struct rcp_msg *msg);

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

    float tf;
    union
    {
        int      i;
        uint8_t  u8;
        uint16_t u16;
        uint32_t u32;
        int8_t   i8;
        int16_t  i16;
        int32_t  i32;
    } temp;

    switch (val->type)
    {
    case DAQ_TYPE_FLOAT:
        tf = *(float*)val->ptr;
        break;
    case DAQ_TYPE_INT:
        temp.i = *(int*)val->ptr;
        tf = (float)temp.i;
        break;
    case DAQ_TYPE_UINT8:
        temp.u8 = *(uint8_t*)val->ptr;
        tf = (float)temp.u8;
        break;
    case DAQ_TYPE_UINT16:
        temp.u16 = *(uint16_t*)val->ptr;
        tf = (float)temp.u16;
        break;
    case DAQ_TYPE_UINT32:
        temp.u32 = *(uint32_t*)val->ptr;
        tf = (float)temp.u32;
        break;
    case DAQ_TYPE_INT8:
        temp.i8 = *(int8_t*)val->ptr;
        tf = (float)temp.i8;
        break;
    case DAQ_TYPE_INT16:
        temp.i16 = *(int16_t*)val->ptr;
        tf = (float)temp.i16;
        break;
    case DAQ_TYPE_INT32:
        temp.i32 = *(int32_t*)val->ptr;
        tf = (float)temp.i32;
        break;
    }
    *out = tf;

    return RES_OK;
}

//-----------------------------------------------------------------

static void rcp_daq_list_cb(struct rcp_msg *msg)
{
    (void)msg;

    struct rcp_msg resp;
    resp.cmd    = RCP_CMD_DAQ_LIST;
    resp.query  = RCP_CMD_OK;

    resp.daq_list.count = values_num;

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void rcp_daq_info_cb(struct rcp_msg *msg)
{
    struct rcp_msg resp;
    resp.cmd = RCP_CMD_DAQ_INFO;

    const struct daq_value *val = daq_get_info(msg->daq_info.id);
    if (!val)
    {
        resp.query = RCP_CMD_ERROR;
    }
    else
    {
        strncpy(resp.daq_info.name, val->name, DAQ_NAME_MAX_LEN);
        strncpy(resp.daq_info.unit, val->unit, DAQ_UNIT_MAX_LEN);
        resp.daq_info.id = msg->daq_info.id;
        resp.query = RCP_CMD_OK;
    }

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void rcp_daq_get_cb(struct rcp_msg *msg)
{
    struct rcp_msg resp;
    resp.cmd    = RCP_CMD_DAQ_GET;
    resp.query  = RCP_CMD_OK;

    uint8_t num = msg->daq_get_req.num;
    if (num > DAQ_GET_VALS_MAX)
        num = DAQ_GET_VALS_MAX;

    resp.daq_get.time = (uint32_t)xTaskGetTickCount();
    resp.daq_get.num = num;

    uint8_t i;
    for (i = 0; i < num; i++)
    {
        result_t res = daq_get_value(msg->daq_get_req.ids[i], &resp.daq_get.values[i]);
        if (res != RES_OK)
        {
            resp.query = RCP_CMD_ERROR;
            break;
        }
    }

    rcp_send_message(&resp);
}
