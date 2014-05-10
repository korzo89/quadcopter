#include <stellaris_config.h>

#include "comm.h"
#include "timers.h"

#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <portmacro.h>
#include <task.h>
#include <semphr.h>

#include <utils/ustdlib.h>
#include <utils/uartstdio.h>

#include <utils/delay.h>
#include <drivers/common_i2c.h>
#include <drivers/oled.h>
#include <drivers/eeprom.h>
#include <drivers/buzzer.h>
#include <drivers/led.h>
#include <drivers/motors.h>
#include <drivers/ultrasonic.h>
#include <drivers/adc.h>
#include <modules/imu.h>
#include <modules/gps.h>

//-----------------------------------------------------------------

#define BUTTON_READ()      GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0)
#define BUTTON_PRESSED()   (!BUTTON_READ())

//-----------------------------------------------------------------

static xSemaphoreHandle mutex;
static xQueueHandle buttonQueue;

static int ledCounter = 0;
static int buttonCounter = 0;

//-----------------------------------------------------------------

static void ledTask(void *params)
{
    while (1)
    {
        ledSet(LED_RED);
        vTaskDelay(MSEC_TO_TICKS(150));
        ledSet(LED_YELLOW);
        vTaskDelay(MSEC_TO_TICKS(150));
        ledSet(LED_GREEN);
        vTaskDelay(MSEC_TO_TICKS(150));

        xSemaphoreTake(mutex, portMAX_DELAY);
        ++ledCounter;
        xSemaphoreGive(mutex);
    }
}

static void oledTask(void *params)
{
    uint8_t screen = 0;
    char buf[17];

    oledClear();
    oledDispStr("Hello world!");
    vTaskDelay(MSEC_TO_TICKS(1000));

    uint8_t i, state;
    float dist;
    float battery;

    GpsMessage msg;

    const int NUM_SCREENS = 4;

    while (1)
    {
        if (xQueueReceive(buttonQueue, &state, 0))
        {
            switch (state)
            {
            case 0:
                break;
            case 1:
                ++buttonCounter;
                break;
            case 2:
                screen = (screen + 1) % NUM_SCREENS;

                oledClear();

                buzzerSetFreq(NOTE_C5);
                vTaskDelay(MSEC_TO_TICKS(10));
                buzzerSetFreq(0);
                break;
            default:
                break;
            }
        }

        usprintf(buf, "      %d/%d       ", screen + 1, NUM_SCREENS);
        oledDispStrAt(buf, 0, 0);

        switch (screen)
        {
        case 0:
            xSemaphoreTake(mutex, portMAX_DELAY);
            usprintf(buf, "Count: %4d", ledCounter);
            xSemaphoreGive(mutex);
            oledDispStrAt(buf, 2, 0);

            usprintf(buf, "Button: %3d", buttonCounter);
            oledDispStrAt(buf, 3, 0);
            break;

        case 1:
            oledSetPos(6, 0);
            state = ledCounter % 16;
            for (i = 0; i < state; ++i)
                oledDispChar('_');
            oledDispChar('*');
            for (i = state + 1; i < 16; ++i)
                oledDispChar('_');

            oledSetPos(7, 0);
            state = buttonCounter % 16;
            for (i = 0; i < state; ++i)
                oledDispChar('_');
            oledDispChar('*');
            for (i = state + 1; i < 16; ++i)
                oledDispChar('_');
            break;

        case 2:
            dist = ultrasonicGetDistance();
            usprintf(buf, "Dist: %7d mm", (int)(dist * 10));
            oledDispStrAt(buf, 2, 0);

            battery = (float)adcGetValue() * 3.3 / 4095.0 * 78.0 / 10.0;
            usprintf(buf, "Battery: %2d.%1d V", (int)battery, (int)(battery * 10) % 10);
            oledDispStrAt(buf, 3, 0);

            break;

        case 3:
            oledDispStrAt("GPS NMEA", 2, 0);

            if (gpsGetMessage(&msg))
            {
                oledClearRect(3, 0, 128, 5);
                oledDispStrAt((char*)msg.command, 3, 0);
                oledDispStrAt((char*)msg.data, 4, 0);
            }
            break;

        default:
            break;
        }

        vTaskDelay(MSEC_TO_TICKS(100));
    }
}

static void buttonTask(void *params)
{
    uint8_t state = 0, curr;
    long lastPress = 0;

    while (1)
    {
        if (state && xTaskGetTickCount() - lastPress >= MSEC_TO_TICKS(600))
        {
            lastPress = xTaskGetTickCount();
            curr = 2;
            xQueueSendToBack(buttonQueue, &curr, 0);
        }

        if (BUTTON_PRESSED() != state)
        {
            vTaskDelay(20);

            curr = BUTTON_PRESSED();
            if (curr != state)
            {
                state = curr;
                xQueueSendToBack(buttonQueue, &curr, portMAX_DELAY);

                lastPress = xTaskGetTickCount();
            }
        }

        vTaskDelay(20);
    }
}

xQueueHandle uartQueue;

static void gpsTask(void *params)
{
    char c;

    while (1)
    {
        if (xQueueReceive(uartQueue, &c, portMAX_DELAY))
        {
            UARTprintf("%c", c);
            gpsParseNMEAChar(c);
        }
    }
}

void UART2IntHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    unsigned long status = UARTIntStatus(UART2_BASE, true);
    UARTIntClear(UART2_BASE, status);

    if (status & UART_INT_RX)
    {
        char c;
        while (UARTCharsAvail(UART2_BASE))
        {
            c = (char)UARTCharGetNonBlocking(UART2_BASE);
            xQueueSendToBackFromISR(uartQueue, &c, &xHigherPriorityTaskWoken);
        }
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void initTask(void *params)
{
    oledInit();

    if (BUTTON_PRESSED())
    {
        char buf[17];

        ledSet(LED_GREEN);
        buzzerSetFreq(NOTE_C5);
        vTaskDelay(MSEC_TO_TICKS(100));
        ledSet(LED_GREEN | LED_YELLOW);
        buzzerSetFreq(NOTE_D5);
        vTaskDelay(MSEC_TO_TICKS(100));
        ledSet(LED_GREEN | LED_YELLOW | LED_RED);
        buzzerSetFreq(NOTE_E5);
        vTaskDelay(MSEC_TO_TICKS(100));
        buzzerSetFreq(0);

        oledClear();
        oledDispStr("EEPROM read\n");

        uint8_t eep[4] = { 0 };
        eepromRead(0x0000, eep, 4);

        usprintf(buf, "%02x %02x %02x %02x", eep[0], eep[1], eep[2], eep[3]);
        oledDispStr(buf);

        while (BUTTON_PRESSED());
        while (!BUTTON_PRESSED());
        while (BUTTON_PRESSED());
    }

    xTaskCreate(ledTask, (signed portCHAR*)"LED",
                configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(oledTask, (signed portCHAR*)"OLED",
                256, NULL, 2, NULL);
    xTaskCreate(buttonTask, (signed portCHAR*)"BTN",
                configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(gpsTask, (signed portCHAR*)"GPS",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskDelete(NULL);
}

//-----------------------------------------------------------------

void main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
            SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    IntMasterEnable();

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    uartQueue = xQueueCreate(256, sizeof(char));

    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 9600,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    UARTIntEnable(UART2_BASE, UART_INT_RX);
    IntPrioritySet(INT_UART2, configKERNEL_INTERRUPT_PRIORITY);
    IntEnable(INT_UART2);

//    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    UARTStdioInit(0);

    adcConfig();
    ultrasonicConfig();
    buzzerConfig();
    ledConfig();

    ledSet(LED_GREEN);

    motorsConfig();
    motorsInit();

    motorsSetThrottle(500, 1000, 333, 0);

    comI2CConfig();

    mutex = xSemaphoreCreateMutex();

    buttonQueue = xQueueCreate(10, sizeof(uint8_t));

    xTaskCreate(initTask, (signed portCHAR*)"INIT",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1)
    {
    }

//    commConfig();
//
//    IMUConfig();
//    IMUInit(0.1f, 100.0f);
//
//    timersConfig();
//
//    while (1)
//    {
//        commPollReceiver();
//    }
}
