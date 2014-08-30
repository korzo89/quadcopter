/*
 * rcp_cmd.h
 *
 *  Created on: 16 sie 2014
 *      Author: Korzo
 */

#ifndef RCP_CMD_H_
#define RCP_CMD_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

#define RCP_PAYLOAD_SIZE        32

#define IS_VALID_CMD(x)         ( (int)(x) >= (int)RCP_CMD_OK && (int)(x) < (int)RCP_CMD_NUM )

//-----------------------------------------------------------------

enum rcp_cmd
{
    RCP_CMD_OK = 0,
    RCP_CMD_ERROR,

    RCP_CMD_CONTROL,
    RCP_CMD_RAW_IMU,
    RCP_CMD_ANGLES,
    RCP_CMD_PID,

    RCP_CMD_PARAM_LIST,
    RCP_CMD_PARAM_INFO,
    RCP_CMD_PARAM_GET,
    RCP_CMD_PARAM_SET,
    RCP_CMD_PARAM_SAVE,

    RCP_CMD_DAQ_LIST,
    RCP_CMD_DAQ_INFO,
    RCP_CMD_DAQ_GET,

    RCP_CMD_LIMITS,
    RCP_CMD_AXIS_MODES,

    RCP_CMD_NUM
};


struct control_value
{
    float   value;
    uint8_t absolute;
} PACK_STRUCT;

struct control_axis
{
    struct control_value value;
    struct
    {
        uint8_t override    : 1;
        uint8_t mode        : 7;
    } mode;
} PACK_STRUCT;

struct cmd_control
{
    struct
    {
        uint8_t armed       : 1;
        uint8_t reserved    : 7;
    }  flags;
    struct control_value    throttle;
    struct control_axis     pitch;
    struct control_axis     roll;
    struct control_axis     yaw;
} PACK_STRUCT;


struct sensor_vec3_pack
{
    int16_t x, y, z;
} PACK_STRUCT;

struct cmd_raw_imu
{
    struct sensor_vec3_pack acc;
    struct sensor_vec3_pack gyro;
    struct sensor_vec3_pack mag;
    int32_t pressure;
    int16_t temperature;
} PACK_STRUCT;


struct vec3_pack
{
    float x, y, z;
} PACK_STRUCT;

struct cmd_angles
{
    struct vec3_pack angles;
    struct vec3_pack rates;
} PACK_STRUCT;


struct cmd_pid
{
    uint8_t type;
    float   kp;
    float   ki;
    float   kd;
    float   kt;
} PACK_STRUCT;


struct cmd_param_list
{
    uint8_t count;
} PACK_STRUCT;


#define PARAM_MAX_GROUP_LEN     15
#define PARAM_MAX_NAME_LEN      11

struct cmd_param_info
{
    uint8_t id;
    char    group[PARAM_MAX_GROUP_LEN];
    char    name[PARAM_MAX_NAME_LEN];
    uint8_t type;
    uint8_t size;
    uint8_t count;
} PACK_STRUCT;

struct cmd_param_data
{
    uint8_t id;
    uint8_t offset;
    uint8_t count;
    uint8_t data[RCP_PAYLOAD_SIZE - 5];
};


#define DAQ_NAME_MAX_LEN    22
#define DAQ_UNIT_MAX_LEN    7

#define DAQ_GET_VALS_MAX    6

struct cmd_daq_list
{
    uint8_t count;
} PACK_STRUCT;

struct cmd_daq_info
{
    uint8_t id;
    char    name[DAQ_NAME_MAX_LEN];
    char    unit[DAQ_UNIT_MAX_LEN];
} PACK_STRUCT;

struct cmd_daq_get_req
{
    uint8_t num;
    uint8_t ids[DAQ_GET_VALS_MAX];
} PACK_STRUCT;

struct cmd_daq_get
{
    uint32_t    time;
    uint8_t     num;
    float       values[DAQ_GET_VALS_MAX];
} PACK_STRUCT;

#define LIMITS_MAX      3

struct control_limit_packed
{
    float limit;
    float dead_zone;
} PACK_STRUCT;

struct limit_data
{
    uint8_t valid;
    uint8_t type;
    struct control_limit_packed limit;
} PACK_STRUCT;

struct cmd_limits
{
    struct limit_data limits[LIMITS_MAX];
} PACK_STRUCT;


struct cmd_axis_modes
{
    uint8_t pitch;
    uint8_t roll;
    uint8_t yaw;
} PACK_STRUCT;


struct rcp_msg
{
    uint8_t cmd;
    uint8_t query;
    union
    {
        uint8_t raw[RCP_PAYLOAD_SIZE - 2];
        struct cmd_control      control;
        struct cmd_raw_imu      raw_imu;
        struct cmd_angles       angles;
        struct cmd_pid          pid;
        struct cmd_param_list   param_list;
        struct cmd_param_info   param_info;
        struct cmd_param_data   param_data;
        struct cmd_daq_list     daq_list;
        struct cmd_daq_info     daq_info;
        struct cmd_daq_get_req  daq_get_req;
        struct cmd_daq_get      daq_get;
        struct cmd_limits       limits;
        struct cmd_axis_modes   axis_modes;
    };
} PACK_STRUCT;

//-----------------------------------------------------------------

#endif /* RCP_CMD_H_ */
