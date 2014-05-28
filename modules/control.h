/*
 * control.h
 *
 *  Created on: 28 maj 2014
 *      Author: Korzo
 */

#ifndef CONTROL_H_
#define CONTROL_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

typedef struct PACK_STRUCT
{
    uint16_t throttle;
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
    struct
    {
        uint8_t sw1 : 1;
        uint8_t sw2 : 2;
        uint8_t sw3 : 3;
    } flags;
} control_t;

//-----------------------------------------------------------------

result_t control_init(void);

result_t control_get_current(control_t *out);

//-----------------------------------------------------------------

#endif /* CONTROL_H_ */
