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

struct gps_msg
{
    const char *command;
    const char *data;
};

//-----------------------------------------------------------------

void gps_init(void);

bool gps_parse_nmea(const char *data, uint32_t len);
bool gps_parse_nmea_char(char c);

bool gps_get_message(struct gps_msg *msg);

void gps_task(void *params);

//-----------------------------------------------------------------

#endif /* GPS_H_ */
