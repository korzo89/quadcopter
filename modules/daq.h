/*
 * daq.h
 *
 *  Created on: 26 cze 2014
 *      Author: Korzo
 */

#ifndef DAQ_H_
#define DAQ_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

#define DAQ_NAME_MAX_LEN    22
#define DAQ_UNIT_MAX_LEN    7

//-----------------------------------------------------------------

enum daq_type
{
    DAQ_TYPE_FLOAT,
    DAQ_TYPE_INT,
    DAQ_TYPE_UINT8,
    DAQ_TYPE_UINT16,
    DAQ_TYPE_UINT32,
    DAQ_TYPE_INT8,
    DAQ_TYPE_INT16,
    DAQ_TYPE_INT32
};

struct daq_value
{
    const char      *name;
    const char      *unit;
    enum daq_type   type;
    void            *ptr;
};

//-----------------------------------------------------------------

result_t daq_init(void);

result_t daq_register_value(const char *name, const char *unit, void *ptr, enum daq_type type);

struct daq_value* daq_get_info(uint8_t id);

result_t daq_get_value(uint8_t id, float *out);

//-----------------------------------------------------------------

#endif /* DAQ_H_ */
