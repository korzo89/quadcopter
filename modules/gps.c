/*
 * gps.c
 *
 *  Created on: 08-05-2014
 *      Author: Korzo
 */

#include "gps.h"

#include <stellaris_config.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <string.h>

//-----------------------------------------------------------------

#define GPS_MAX_CMD_LEN         8
#define GPS_MAX_DATA_LEN        256

#define GPS_UART_BAUD           9600
#define GPS_UART_BUFFER_LEN		256

//-----------------------------------------------------------------

enum gps_parser_state
{
    GPS_STATE_SOM,
    GPS_STATE_CMD,
    GPS_STATE_DATA,
    GPS_STATE_CHECKSUM1,
    GPS_STATE_CHECKSUM2
};

struct gps_obj
{
    char        uart_buffer[GPS_UART_BUFFER_LEN];
    uint32_t    uart_start;
    uint32_t    uart_end;

    enum gps_parser_state parser_state;
    char        command[GPS_MAX_CMD_LEN];
    char        data[GPS_MAX_DATA_LEN];
    uint32_t    index;
    uint8_t     checksum;
    uint8_t     recv_checksum;
    bool        has_message;

    xSemaphoreHandle uart_ready;
};

static struct gps_obj gps;

//-----------------------------------------------------------------

static void gps_task(void *params);

//-----------------------------------------------------------------

void gps_init(void)
{
    memset(&gps, 0, sizeof(gps));

    vSemaphoreCreateBinary(gps.uart_ready);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), GPS_UART_BAUD,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    UARTIntEnable(UART2_BASE, UART_INT_RX);
    IntPrioritySet(INT_UART2, configKERNEL_INTERRUPT_PRIORITY);
    IntEnable(INT_UART2);

    xTaskCreate(gps_task, TASK_NAME("GPS"),
            GPS_TASK_STACK, NULL, GPS_TASK_PRIORITY, NULL);
}

//-----------------------------------------------------------------

void UART2IntHandler(void)
{
    portBASE_TYPE woken = pdFALSE;

    unsigned long status = UARTIntStatus(UART2_BASE, true);
    UARTIntClear(UART2_BASE, status);

    if (status & UART_INT_RX)
    {
        while (UARTCharsAvail(UART2_BASE))
        {
            gps.uart_buffer[gps.uart_end] = (char)UARTCharGetNonBlocking(UART2_BASE);
            gps.uart_end = (gps.uart_end + 1) % GPS_UART_BUFFER_LEN;
        }

        xSemaphoreGiveFromISR(gps.uart_ready, &woken);
    }

    portEND_SWITCHING_ISR(woken);
}

//-----------------------------------------------------------------

static void gps_task(void *params)
{
    xSemaphoreTake(gps.uart_ready, 0);

    while (1)
    {
        if (xSemaphoreTake(gps.uart_ready, portMAX_DELAY) == pdTRUE)
        {
            int end = gps.uart_end;
            while (gps.uart_start != end)
            {
                gps_parse_nmea_char(gps.uart_buffer[gps.uart_start]);
                gps.uart_start = (gps.uart_start + 1) % GPS_UART_BUFFER_LEN;
            }

            xSemaphoreTake(gps.uart_ready, 0);
        }
    }
}

//-----------------------------------------------------------------

bool gps_parse_nmea(const char *data, uint32_t len)
{
    bool res = false;

    uint32_t i;
    for (i = 0; i < len; ++i)
        res |= gps_parse_nmea_char(data[i]);

    return res;
}

//-----------------------------------------------------------------

bool gps_parse_nmea_char(char c)
{
    switch (gps.parser_state)
    {
    case GPS_STATE_SOM:
        if (c == '$')
        {
            gps.checksum = 0;
            gps.index = 0;
            gps.has_message = false;
            gps.parser_state = GPS_STATE_CMD;
        }
        break;

    case GPS_STATE_CMD:
        gps.checksum ^= c;
        if (c != ',' && c != '*')
        {
            gps.command[gps.index++] = c;
            if (gps.index >= GPS_MAX_CMD_LEN)
                gps.parser_state = GPS_STATE_SOM;
        }
        else
        {
            gps.command[gps.index] = '\0';
            gps.index = 0;
            gps.parser_state = GPS_STATE_DATA;
        }
        break;

    case GPS_STATE_DATA:
        if (c == '*')
        {
            gps.data[gps.index] = '\0';
            gps.parser_state = GPS_STATE_CHECKSUM1;
        }
        else
        {
            if (c == '\r')
            {
                gps.data[gps.index] = '\0';
                gps.parser_state = GPS_STATE_SOM;
                gps.has_message = true;
                return true;
            }

            gps.checksum ^= c;
            gps.data[gps.index++] = c;
            if (gps.index >= GPS_MAX_DATA_LEN)
                gps.parser_state = GPS_STATE_SOM;
        }
        break;

    case GPS_STATE_CHECKSUM1:
        if ((c - '0') <= 9)
            gps.recv_checksum = (c - '0') << 4;
        else
            gps.recv_checksum = (c - 'A' + 10) << 4;

        gps.parser_state = GPS_STATE_CHECKSUM2;
        break;

    case GPS_STATE_CHECKSUM2:
        if ((c - '0') <= 9)
            gps.recv_checksum |= c - '0';
        else
            gps.recv_checksum |= c - 'A' + 10;

        gps.parser_state = GPS_STATE_SOM;

        if (gps.checksum == gps.recv_checksum)
        {
            gps.has_message = true;
            return true;
        }

        break;
    }

    return false;
}

//-----------------------------------------------------------------

bool gps_get_message(struct gps_msg *msg)
{
    if (!gps.has_message)
        return false;

    msg->command = (const char*)gps.command;
    msg->data = (const char*)gps.data;
    return true;
}


