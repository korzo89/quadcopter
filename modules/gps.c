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

static char uart_buffer[GPS_UART_BUFFER_LEN];
static int uart_start = 0, uart_end = 0;

static enum gps_parser_state parser_state = GPS_STATE_SOM;
static char command[GPS_MAX_CMD_LEN] = "";
static char data[GPS_MAX_DATA_LEN] = "";
static int index;
static uint8_t checksum, recv_checksum;
static bool has_message = false;

static xSemaphoreHandle uart_ready;

//-----------------------------------------------------------------

void gps_init(void)
{
    vSemaphoreCreateBinary(uart_ready);

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
        	uart_buffer[uart_end] = (char)UARTCharGetNonBlocking(UART2_BASE);
        	uart_end = (uart_end + 1) % GPS_UART_BUFFER_LEN;
        }

        xSemaphoreGiveFromISR(uart_ready, &woken);
    }

    portEND_SWITCHING_ISR(woken);
}

//-----------------------------------------------------------------

void gps_task(void *params)
{
    xSemaphoreTake(uart_ready, 0);

    while (1)
    {
        if (xSemaphoreTake(uart_ready, portMAX_DELAY) == pdTRUE)
        {
            int end = uart_end;
            while (uart_start != end)
            {
                gps_parse_nmea_char(uart_buffer[uart_start]);
                uart_start = (uart_start + 1) % GPS_UART_BUFFER_LEN;
            }

            xSemaphoreTake(uart_ready, 0);
        }
    }
}

//-----------------------------------------------------------------

bool gps_parse_nmea(const char *data, unsigned int len)
{
    bool res = false;

    unsigned int i;
    for (i = 0; i < len; ++i)
        res |= gps_parse_nmea_char(data[i]);

    return res;
}

//-----------------------------------------------------------------

bool gps_parse_nmea_char(char c)
{
    switch (parser_state)
    {
    case GPS_STATE_SOM:
        if (c == '$')
        {
            checksum = 0;
            index = 0;
            has_message = false;
            parser_state = GPS_STATE_CMD;
        }
        break;

    case GPS_STATE_CMD:
        checksum ^= c;
        if (c != ',' && c != '*')
        {
            command[index++] = c;
            if (index >= GPS_MAX_CMD_LEN)
                parser_state = GPS_STATE_SOM;
        }
        else
        {
            command[index] = '\0';
            index = 0;
            parser_state = GPS_STATE_DATA;
        }
        break;

    case GPS_STATE_DATA:
        if (c == '*')
        {
            data[index] = '\0';
            parser_state = GPS_STATE_CHECKSUM1;
        }
        else
        {
            if (c == '\r')
            {
                data[index] = '\0';
                parser_state = GPS_STATE_SOM;
                has_message = true;
                return true;
            }

            checksum ^= c;
            data[index++] = c;
            if (index >= GPS_MAX_DATA_LEN)
                parser_state = GPS_STATE_SOM;
        }
        break;

    case GPS_STATE_CHECKSUM1:
        if ((c - '0') <= 9)
            recv_checksum = (c - '0') << 4;
        else
            recv_checksum = (c - 'A' + 10) << 4;

        parser_state = GPS_STATE_CHECKSUM2;
        break;

    case GPS_STATE_CHECKSUM2:
        if ((c - '0') <= 9)
            recv_checksum |= c - '0';
        else
            recv_checksum |= c - 'A' + 10;

        parser_state = GPS_STATE_SOM;

        if (checksum == recv_checksum)
        {
            has_message = true;
            return true;
        }

        break;
    }

    return false;
}

//-----------------------------------------------------------------

bool gps_get_message(struct gps_msg *msg)
{
    if (!has_message)
        return false;

    msg->command = (const char*)command;
    msg->data = (const char*)data;
    return true;
}


