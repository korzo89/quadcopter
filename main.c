#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"

#include "comm.h"
#include "motors.h"
#include "led.h"
#include "imu/imu.h"
#include "timers.h"
#include "buzzer_notes.h"

#include <FreeRTOS.h>
#include <portmacro.h>
#include <task.h>
#include <semphr.h>

#include <utils/ustdlib.h>

#include <utils/delay.h>
#include <drivers/common_i2c.h>
#include <drivers/oled.h>
#include <drivers/eeprom.h>

//-----------------------------------------------------------------

static xSemaphoreHandle mutex;
static xQueueHandle buttonQueue;

static bool initialized = false;
static xSemaphoreHandle initMutex;

static int ledCounter = 0;
static int buttonCounter = 0;

const unsigned int notes[16] = {
        0, NOTE_C4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4,
        NOTE_C5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_D5
};

static void buzzerSetFrequency(unsigned int freq)
{
    uint32_t cycle = SysCtlClockGet() / freq;
    uint32_t period = cycle;
//    uint8_t ext = period >> 16;
//    period &= 0xFFFF;

    TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
    TimerControlLevel(WTIMER1_BASE, TIMER_A, false);
    //TimerPrescaleSet(WTIMER1_BASE, TIMER_A, ext);
    TimerPrescaleSet(WTIMER1_BASE, TIMER_A, 0);
    TimerLoadSet(WTIMER1_BASE, TIMER_A, period);

    period = cycle * 4 / 5;
//    ext = period >> 16;
//    period &= 0xFFFF;

    TimerMatchSet(WTIMER1_BASE, TIMER_A, period);
    TimerPrescaleMatchSet(WTIMER1_BASE, TIMER_A, 0);
    //TimerPrescaleMatchSet(WTIMER1_BASE, TIMER_A, ext);

    if (freq != 0)
        TimerEnable(WTIMER1_BASE, TIMER_A);
    else
        TimerDisable(WTIMER1_BASE, TIMER_A);
}

static void initWait(void)
{
    while (!initialized)
        vTaskDelay(2);

    xSemaphoreTake(initMutex, portMAX_DELAY);
    xSemaphoreGive(initMutex);
}

static void initTask(void *params)
{
    initMutex = xSemaphoreCreateMutex();

    xSemaphoreTake(initMutex, portMAX_DELAY);

    oledInit();

    if (GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0))
    {
        initialized = true;
        xSemaphoreGive(initMutex);
        vTaskDelete(NULL);
        return;
    }

    LEDSet(LED_GREEN);
    buzzerSetFrequency(NOTE_C4);
    vTaskDelay(MSEC_TO_TICKS(100));
    LEDSet(LED_GREEN | LED_YELLOW);
    buzzerSetFrequency(NOTE_D4);
    vTaskDelay(MSEC_TO_TICKS(100));
    LEDSet(LED_GREEN | LED_YELLOW | LED_RED);
    buzzerSetFrequency(NOTE_E4);
    vTaskDelay(MSEC_TO_TICKS(100));
    buzzerSetFrequency(0);

    oledClear();
    oledDispStr("EEPROM read\n");

    uint8_t eep[4] = { 0 };
    eepromRead(0x0000, eep, 4);

    char buf[17];
    usprintf(buf, "%02x %02x %02x %02x", eep[0], eep[1], eep[2], eep[3]);
    oledDispStr(buf);

    while (!GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0));
    while (GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0));
    while (!GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0));

    initialized = true;
    xSemaphoreGive(initMutex);
    vTaskDelete(NULL);
}

static void ledTask(void *params)
{
    initWait();

    while (1)
    {
        LEDSet(LED_RED);
        vTaskDelay(MSEC_TO_TICKS(150));
        LEDSet(LED_YELLOW);
        vTaskDelay(MSEC_TO_TICKS(150));
        LEDSet(LED_GREEN);
        vTaskDelay(MSEC_TO_TICKS(150));

        xSemaphoreTake(mutex, portMAX_DELAY);
        ++ledCounter;
        xSemaphoreGive(mutex);
    }
}

static void oledTask(void *params)
{
    initWait();

    uint8_t screen = 0;
    char buf[17];

    oledClear();
    oledDispStr("Hello world!");
    vTaskDelay(MSEC_TO_TICKS(1000));

    uint8_t i, state;
//    unsigned int freq;

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

//                freq = notes[buttonCounter % 16];
//                if (freq == 0)
//                {
//                    TimerDisable(WTIMER1_BASE, TIMER_A);
//                }
//                else
//                {
//                    buzzerSetFrequency(freq);
//                    TimerEnable(WTIMER1_BASE, TIMER_A);
//                }
                break;
            case 2:
                screen = (screen + 1) % 3;

                oledClear();

                buzzerSetFrequency(NOTE_C5);
                vTaskDelay(MSEC_TO_TICKS(10));
                buzzerSetFrequency(0);
                break;
            default:
                break;
            }
        }

        usprintf(buf, "      %d/3       ", screen + 1);
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
            oledSetPos(2, 0);
            while (UARTCharsAvail(UART2_BASE))
                oledDispChar((char)UARTCharGetNonBlocking(UART2_BASE));
            break;

        default:
            break;
        }

        vTaskDelay(MSEC_TO_TICKS(100));
    }
}

static void buttonTask(void *params)
{
    initWait();

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

        if (GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0) == state)
        {
            vTaskDelay(20);

            curr = !GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0);
            if (curr != state)
            {
                state = curr;
                xQueueSendToBack(buttonQueue, &curr, 0);

                lastPress = xTaskGetTickCount();
            }
        }

        vTaskDelay(20);
    }
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

    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 9600,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    // buzzer timer config
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);

    GPIOPinConfigure(GPIO_PC6_WT1CCP0);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6);

    buzzerSetFrequency(0);

//    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    LEDConfig();
    LEDSet(LED_GREEN);

    motorsConfig();
    motorsInit();

    motorsSetThrottle(500, 1000, 333, 0);

    comI2CConfig();

    mutex = xSemaphoreCreateMutex();

    buttonQueue = xQueueCreate(10, sizeof(uint8_t));

    xTaskCreate(initTask, (signed portCHAR*)"INIT", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(ledTask, (signed portCHAR*)"LED", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(oledTask, (signed portCHAR*)"OLED", 256,
                NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(buttonTask, (signed portCHAR*)"BTN", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while (1)
    {
    }

//
//    UARTStdioInit(0);
//
//    commConfig();
//
//    IMUConfig();
//    IMUInit(0.1f, 100.0f);
//
//    oledConfig();
//    if (oledInit())
//        oledDispStr("Hello world!");
//
//    timersConfig();
//
//    while (1)
//    {
//        commPollReceiver();
//    }
}

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}
