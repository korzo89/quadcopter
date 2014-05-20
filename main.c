#include <stellaris_config.h>

#include "timers.h"

#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <portmacro.h>
#include <task.h>
#include <semphr.h>

#include <utils/ustdlib.h>
#include <utils/uartstdio.h>

#include <utils/delay.h>
#include <drivers/ext_i2c.h>
#include <drivers/oled.h>
#include <drivers/eeprom.h>
#include <drivers/buzzer.h>
#include <drivers/led.h>
#include <drivers/motors.h>
#include <drivers/ultrasonic.h>
#include <drivers/adc.h>
#include <drivers/adxl345.h>
#include <drivers/l3g4200d.h>
#include <drivers/hmc5883.h>
#include <drivers/bmp085.h>
#include <modules/imu.h>
#include <modules/gps.h>
#include <modules/rcp.h>

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
    	ledTurnOn(LED_GREEN);
        vTaskDelay(MSEC_TO_TICKS(150));
        ledTurnOff(LED_GREEN);
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

    oled_clear();
    oled_disp_str("Hello world!");
    vTaskDelay(MSEC_TO_TICKS(1000));

    uint8_t i, state;
    float dist;
    float battery;

    static GpsMessage msg;
    static IMUSensorData sensors;

    const int NUM_SCREENS = 8;

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

                oled_clear();

                buzzerSetFreq(NOTE_C5);
                vTaskDelay(MSEC_TO_TICKS(10));
                buzzerSetFreq(0);
                break;
            default:
                break;
            }
        }

        usprintf(buf, "      %d/%d       ", screen + 1, NUM_SCREENS);
        oled_disp_str_at(buf, 0, 0);

        switch (screen)
        {
        case 0:
            xSemaphoreTake(mutex, portMAX_DELAY);
            usprintf(buf, "Count: %4d", ledCounter);
            xSemaphoreGive(mutex);
            oled_disp_str_at(buf, 2, 0);

            usprintf(buf, "Button: %3d", buttonCounter);
            oled_disp_str_at(buf, 3, 0);
            break;

        case 1:
            oled_set_pos(6, 0);
            state = ledCounter % 16;
            for (i = 0; i < state; ++i)
                oled_disp_char('_');
            oled_disp_char('*');
            for (i = state + 1; i < 16; ++i)
                oled_disp_char('_');

            oled_set_pos(7, 0);
            state = buttonCounter % 16;
            for (i = 0; i < state; ++i)
                oled_disp_char('_');
            oled_disp_char('*');
            for (i = state + 1; i < 16; ++i)
                oled_disp_char('_');
            break;

        case 2:
            dist = ultrasonicGetDistance();
            usprintf(buf, "Dist: %7d mm", (int)(dist * 10));
            oled_disp_str_at(buf, 2, 0);

            if (buttonCounter % 2) {
                if (dist < 10.0f)
                    buzzerSetFreq(NOTE_E6);
                else if (dist < 20.0f)
                    buzzerSetFreq(NOTE_D6);
                else if (dist < 30.0f)
                    buzzerSetFreq(NOTE_C6);
                else
                    buzzerSetFreq(0);
            } else {
                buzzerSetFreq(0);
            }

            battery = (float)adcGetValue() * 3.3 / 4095.0 * 78.0 / 10.0;
            usprintf(buf, "Battery: %2d.%1d V", (int)battery, (int)(battery * 10) % 10);
            oled_disp_str_at(buf, 3, 0);

            break;

        case 3:
            adxl345GetAcceleration(&sensors.accX, &sensors.accY, &sensors.accZ);
            oled_disp_str_at("Accelerometer", 2, 0);
            usprintf(buf, "X: %5d", sensors.accX);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Y: %5d", sensors.accY);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "Z: %5d", sensors.accZ);
            oled_disp_str_at(buf, 6, 0);
            break;

        case 4:
            l3g4200dReadGyro(&sensors.gyroX, &sensors.gyroY, &sensors.gyroZ);
            oled_disp_str_at("Gyroscope", 2, 0);
            usprintf(buf, "X: %5d", sensors.gyroX);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Y: %5d", sensors.gyroY);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "Z: %5d", sensors.gyroZ);
            oled_disp_str_at(buf, 6, 0);
            break;

        case 5:
            hmc5883ReadMag(&sensors.magX, &sensors.magY, &sensors.magZ);
            oled_disp_str_at("Magnetometer", 2, 0);
            usprintf(buf, "X: %5d", sensors.magX);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Y: %5d", sensors.magY);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "Z: %5d", sensors.magZ);
            oled_disp_str_at(buf, 6, 0);
            break;

        case 6:
            sensors.temperature = bmp085ReadTemperature();
            sensors.pressure = bmp085ReadPressure();
            oled_disp_str_at("Barometer", 2, 0);
            usprintf(buf, "T: %5d", sensors.temperature);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "P: %8d", sensors.pressure);
            oled_disp_str_at(buf, 5, 0);
            break;

        case 7:
            oled_disp_str_at("GPS NMEA", 2, 0);

            if (gpsGetMessage(&msg))
            {
                oled_clear_rect(3, 0, 128, 5);
                oled_disp_str_at((char*)msg.command, 3, 0);
                oled_disp_str_at((char*)msg.data, 4, 0);
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

static void initTask(void *params)
{
    ext_i2c_init();

    eeprom_init();
    oled_init();

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

        oled_clear();
        oled_disp_str("EEPROM read\n");

        uint8_t eep[4] = { 0 };
        eeprom_read(0x0000, eep, 4);

        usprintf(buf, "%02x %02x %02x %02x", eep[0], eep[1], eep[2], eep[3]);
        oled_disp_str(buf);

        while (BUTTON_PRESSED());
        while (!BUTTON_PRESSED());
        while (BUTTON_PRESSED());

        ledSet(LEDS_OFF);
    }

    xTaskCreate(ledTask, (signed portCHAR*)"LED",
                configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(oledTask, (signed portCHAR*)"OLED",
                256, NULL, 2, NULL);
    xTaskCreate(buttonTask, (signed portCHAR*)"BTN",
                configMINIMAL_STACK_SIZE, NULL, 2, NULL);

//    xTaskCreate(commTask, (signed portCHAR*)"COMM",
//                    configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    rcpInit();
    gpsInit();

    vTaskDelete(NULL);
}

//-----------------------------------------------------------------

void main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
            SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    IntMasterEnable();

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    UARTStdioInit(0);

    adcInit();
    ultrasonicInit();
    buzzerInit();
    ledInit();

//    ledSet(LED_GREEN);

    motorsInit();
    motorsSetThrottle(500, 1000, 333, 0);

    imuInit(0.1f, 100.0f);

    mutex = xSemaphoreCreateMutex();
    buttonQueue = xQueueCreate(10, sizeof(uint8_t));

    xTaskCreate(initTask, (signed portCHAR*)"INIT",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1)
    {
    }
}
