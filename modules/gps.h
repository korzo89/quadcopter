/*
 * gps.h
 *
 *  Created on: 08-05-2014
 *      Author: Korzo
 */

#ifndef GPS_H_
#define GPS_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

typedef struct
{
    const char *command;
    const char *data;
} gps_message_t;

//-----------------------------------------------------------------

void gps_init(void);

bool gps_parse_nmea(const char *data, unsigned int len);
bool gps_parse_nmea_char(char c);

bool gps_get_message(gps_message_t *msg);

void gps_task(void *params);

//-----------------------------------------------------------------

#endif /* GPS_H_ */
