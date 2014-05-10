/*
 * gps.c
 *
 *  Created on: 08-05-2014
 *      Author: Korzo
 */

#include "gps.h"

#include <stdint.h>

//-----------------------------------------------------------------

#define GPS_MAX_CMD_LEN         8
#define GPS_MAX_DATA_LEN        256

//-----------------------------------------------------------------

typedef enum
{
    GPS_STATE_SOM,
    GPS_STATE_CMD,
    GPS_STATE_DATA,
    GPS_STATE_CHECKSUM1,
    GPS_STATE_CHECKSUM2
} GpsParserState;

static GpsParserState parserState = GPS_STATE_SOM;
static char command[GPS_MAX_CMD_LEN] = "";
static char data[GPS_MAX_DATA_LEN] = "";
static int index;
static uint8_t checksum, recvChecksum;
static bool hasMessage = false;

//-----------------------------------------------------------------

void gpsConfig(void)
{

}

//-----------------------------------------------------------------

bool gpsParseNMEA(const char *data, unsigned int len)
{
    bool res = false;

    unsigned int i;
    for (i = 0; i < len; ++i)
        res |= gpsParseNMEAChar(data[i]);

    return res;
}

//-----------------------------------------------------------------

bool gpsParseNMEAChar(char c)
{
    switch (parserState)
    {
    case GPS_STATE_SOM:
        if (c == '$')
        {
            checksum = 0;
            index = 0;
            hasMessage = false;
            parserState = GPS_STATE_CMD;
        }
        break;

    case GPS_STATE_CMD:
        checksum ^= c;
        if (c != ',' && c != '*')
        {
            command[index++] = c;
            if (index >= GPS_MAX_CMD_LEN)
                parserState = GPS_STATE_SOM;
        }
        else
        {
            command[index] = '\0';
            index = 0;
            parserState = GPS_STATE_DATA;
        }
        break;

    case GPS_STATE_DATA:
        if (c == '*')
        {
            data[index] = '\0';
            parserState = GPS_STATE_CHECKSUM1;
        }
        else
        {
            if (c == '\r')
            {
                data[index] = '\0';
                parserState = GPS_STATE_SOM;
                hasMessage = true;
                return true;
            }

            checksum ^= c;
            data[index++] = c;
            if (index >= GPS_MAX_DATA_LEN)
                parserState = GPS_STATE_SOM;
        }
        break;

    case GPS_STATE_CHECKSUM1:
        if ((c - '0') <= 9)
            recvChecksum = (c - '0') << 4;
        else
            recvChecksum = (c - 'A' + 10) << 4;

        parserState = GPS_STATE_CHECKSUM2;
        break;

    case GPS_STATE_CHECKSUM2:
        if ((c - '0') <= 9)
            recvChecksum |= c - '0';
        else
            recvChecksum |= c - 'A' + 10;

        parserState = GPS_STATE_SOM;

        if (checksum == recvChecksum)
        {
            hasMessage = true;
            return true;
        }

        break;
    }

    return false;
}

//-----------------------------------------------------------------

bool gpsGetMessage(GpsMessage *msg)
{
    if (!hasMessage)
        return false;

    msg->command = (const char*)command;
    msg->data = (const char*)data;
    return true;
}


