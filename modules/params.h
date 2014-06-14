/*
 * params.h
 *
 *  Created on: 14 cze 2014
 *      Author: Korzo
 */

#ifndef PARAMS_H_
#define PARAMS_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

enum param_type
{
    PARAM_TYPE_UINT8 = 0,
    PARAM_TYPE_UINT16,
    PARAM_TYPE_UINT32,
    PARAM_TYPE_INT8,
    PARAM_TYPE_INT16,
    PARAM_TYPE_INT32,
    PARAM_TYPE_FLOAT
};

typedef struct
{
    const char      *group;     // max group len: 14 chars + zero = 15
    const char      *name;      // max name len: 10 chars + zero = 11
    enum param_type type;
    uint8_t         size;
    uint8_t         count;
    void            *ptr;
} param_info_t;

//-----------------------------------------------------------------

result_t params_init(void);

const param_info_t* params_get_info(uint8_t id);

//-----------------------------------------------------------------

#endif /* PARAMS_H_ */
