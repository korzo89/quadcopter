/*
 * gps.h
 *
 *  Created on: 08-05-2014
 *      Author: Korzo
 */

#ifndef GPS_H_
#define GPS_H_

//-----------------------------------------------------------------

#include <stdbool.h>

//-----------------------------------------------------------------

typedef struct
{
    const char *command;
    const char *data;
} GpsMessage;

//-----------------------------------------------------------------

void gpsConfig(void);

bool gpsParseNMEA(const char *data, unsigned int len);
bool gpsParseNMEAChar(char c);

bool gpsGetMessage(GpsMessage *msg);

//-----------------------------------------------------------------

#endif /* GPS_H_ */
