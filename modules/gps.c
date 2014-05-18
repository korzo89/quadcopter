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
#include <queue.h>
#include <semphr.h>

//-----------------------------------------------------------------

#define GPS_MAX_CMD_LEN         8
#define GPS_MAX_DATA_LEN        256

#define GPS_UART_BAUD           9600
#define GPS_UART_BUFFER_LEN		256

//-----------------------------------------------------------------

typedef enum
{
    GPS_STATE_SOM,
    GPS_STATE_CMD,
    GPS_STATE_DATA,
    GPS_STATE_CHECKSUM1,
    GPS_STATE_CHECKSUM2
} GpsParserState;

static char uartBuffer[GPS_UART_BUFFER_LEN];
static int uartStart = 0, uartEnd = 0;

static GpsParserState parserState = GPS_STATE_SOM;
static char command[GPS_MAX_CMD_LEN] = "";
static char data[GPS_MAX_DATA_LEN] = "";
static int index;
static uint8_t checksum, recvChecksum;
static bool hasMessage = false;

static xQueueHandle uartQueue;
static xSemaphoreHandle uartReady;

//-----------------------------------------------------------------

void gpsInit(void)
{
    uartQueue = xQueueCreate(256, sizeof(char));
    vSemaphoreCreateBinary(uartReady);

    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), GPS_UART_BAUD,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    UARTIntEnable(UART2_BASE, UART_INT_RX);
    IntPrioritySet(INT_UART2, configKERNEL_INTERRUPT_PRIORITY);
    IntEnable(INT_UART2);

    xTaskCreate(gpsTask, (signed portCHAR*)"GPS",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

//-----------------------------------------------------------------

void UART2IntHandler(void)
{
    portBASE_TYPE woken = pdFALSE;

    unsigned long status = UARTIntStatus(UART2_BASE, true);
    UARTIntClear(UART2_BASE, status);

    if (status & UART_INT_RX)
    {
        char c;
        while (UARTCharsAvail(UART2_BASE))
        {
//            c = (char)UARTCharGetNonBlocking(UART2_BASE);
//            xQueueSendToBackFromISR(uartQueue, &c, &woken);
        	uartBuffer[uartEnd] = (char)UARTCharGetNonBlocking(UART2_BASE);
        	uartEnd = (uartEnd + 1) % GPS_UART_BUFFER_LEN;
        }

        xSemaphoreGiveFromISR(uartReady, woken);
    }

    portEND_SWITCHING_ISR(woken);
}

//-----------------------------------------------------------------

void gpsTask(void *params)
{
    char c;

    xSemaphoreTake(uartReady, 0);

    while (1)
    {
    	if (xSemaphoreTake(uartReady, portMAX_DELAY) == pdPASS)
    	{
    		int end = uartEnd;
    		while (uartStart != end)
    		{
    			gpsParseNMEAChar(uartBuffer[uartStart]);
    			uartStart = (uartStart + 1) % GPS_UART_BUFFER_LEN;
    		}

    		xSemaphoreTake(uartReady, 0);
    	}

//        if (xQueueReceive(uartQueue, &c, portMAX_DELAY))
//        {
////            UARTprintf("%c", c);
//            gpsParseNMEAChar(c);
//        }
    }
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


