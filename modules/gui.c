/*
 * gui.c
 *
 *  Created on: 20 maj 2014
 *      Author: Korzo
 */

#include "gui.h"

#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <portmacro.h>
#include <task.h>
#include <semphr.h>

#include <drivers/oled.h>
#include <drivers/adxl345.h>
#include <drivers/l3g4200d.h>
#include <drivers/hmc5883.h>
#include <drivers/bmp085.h>
#include <drivers/ultrasonic.h>
#include <drivers/adc.h>
#include <drivers/led.h>
#include <drivers/buzzer.h>

#include <modules/gps.h>
#include <modules/imu.h>
#include <modules/rcp.h>
#include <modules/control.h>

#include <utils/delay.h>
#include <utils/ustdlib.h>

//-----------------------------------------------------------------

#define GUI_TASK_STACK_SIZE     300
#define GUI_TASK_PRIORITY       2

//-----------------------------------------------------------------

static xSemaphoreHandle mutex;
static xQueueHandle button_queue;

static int led_counter = 0;
static int button_counter = 0;

static char buf[17];

//-----------------------------------------------------------------

static void gui_task(void *params);
static void led_task(void *params);
static void button_task(void *params);

//-----------------------------------------------------------------

result_t gui_init(void)
{
    mutex = xSemaphoreCreateMutex();
    button_queue = xQueueCreate(10, sizeof(uint8_t));

    if (xTaskCreate(led_task, (signed portCHAR*)"LED",
            configMINIMAL_STACK_SIZE, NULL, 2, NULL) != pdPASS)
        return RES_ERR_FATAL;

    if (xTaskCreate(button_task, (signed portCHAR*)"BTN",
            configMINIMAL_STACK_SIZE, NULL, 2, NULL) != pdPASS)
        return RES_ERR_FATAL;

    if (xTaskCreate(gui_task, (signed portCHAR*)"GUI",
            GUI_TASK_STACK_SIZE, NULL, GUI_TASK_PRIORITY, NULL) != pdPASS)
        return RES_ERR_FATAL;

    return RES_OK;
}

//-----------------------------------------------------------------

static void led_task(void *params)
{
    while (1)
    {
        led_turn_on(LED_GREEN);
        DELAY_MS(150);
        led_turn_off(LED_GREEN);
        DELAY_MS(150);

        xSemaphoreTake(mutex, portMAX_DELAY);
        ++led_counter;
        xSemaphoreGive(mutex);
    }
}

//-----------------------------------------------------------------

static void button_task(void *params)
{
    uint8_t state = 0, curr;
    long last_press = 0;

    while (1)
    {
        if (state && xTaskGetTickCount() - last_press >= MSEC_TO_TICKS(600))
        {
            last_press = xTaskGetTickCount();
            curr = 2;
            xQueueSendToBack(button_queue, &curr, 0);
        }

        if (BUTTON_PRESSED() != state)
        {
            DELAY_MS(5);

            curr = BUTTON_PRESSED();
            if (curr != state)
            {
                state = curr;
                xQueueSendToBack(button_queue, &curr, portMAX_DELAY);

                last_press = xTaskGetTickCount();
            }
        }

        DELAY_MS(5);
    }
}

//-----------------------------------------------------------------

static void gui_disp_control(const char *name, uint16_t val, int row)
{
    usprintf(buf, "%5s %4d ", name, val);
    oled_disp_str_at(buf, row, 0);

    const int num = 5;
    uint32_t len = (uint32_t)val * 100 / 4095;
    len = len * num / 100;
    int i;
    for (i = 0; i < num; ++i)
        oled_disp_char(i < len ? '=' : '.');
}

//-----------------------------------------------------------------

const uint8_t SYMB_CONN[] = {
//        0x03, 0x05, 0x09, 0x7F, 0x09, 0x65, 0x03, 0x70,
//        0x00, 0x78, 0x00, 0x7C, 0x00, 0x7E, 0x00, 0x7F
        0x03, 0x05, 0x09, 0x7F, 0x09, 0x05, 0x03, 0x00,
        0x00, 0x1C, 0x00, 0x22, 0x1C, 0x41, 0x22, 0x1C
};

const uint8_t SYMB_LOST[] = {
        0x03, 0x05, 0x09, 0x7F, 0x09, 0x05, 0x03, 0x00,
        0x00, 0x22, 0x14, 0x08, 0x14, 0x22, 0x00, 0x00
};

static void gui_task(void *params)
{
    uint8_t screen = 0;

    oled_clear();
    oled_disp_str("Hello world!");
    DELAY_MS(1000);

    uint8_t i, state;
    float dist;
    float battery;

    static gps_message_t msg;
    static imu_sensor_data_t sensors;
    static imu_real_t real_sens;
    static vec3_t angles;

    const int NUM_SCREENS = 8;

    while (1)
    {
        if (xQueueReceive(button_queue, &state, 0))
        {
            switch (state)
            {
            case 0:
                break;
            case 1:
                ++button_counter;
                break;
            case 2:
                screen = (screen + 1) % NUM_SCREENS;

                oled_clear();

                buzzer_set_freq(NOTE_C5);
                DELAY_MS(10);
                buzzer_set_freq(0);
                break;
            default:
                break;
            }
        }

        oled_set_pos(0, 0);
        oled_disp_symbol((uint8_t*)(rcp_is_connected() ? SYMB_CONN : SYMB_LOST), 16);
        usprintf(buf, "    %d/%d       ", screen + 1, NUM_SCREENS);
//        oled_disp_str_at(buf, 0, 0);
        oled_disp_str(buf);

        control_t control;

        switch (screen)
        {
        case 0:
            control_get_current(&control);

            gui_disp_control("1 Thr", control.throttle, 2);
            gui_disp_control("2 Pit", control.pitch, 3);
            gui_disp_control("3 Rol", control.roll, 4);
            gui_disp_control("4 Yaw", control.yaw, 5);
            oled_set_pos(6, 0);
            oled_disp_char(control.flags.sw1 ? 'X' : '_');
            oled_disp_char(control.flags.sw2 ? 'X' : '_');
            oled_disp_char(control.flags.sw3 ? 'X' : '_');
            break;

        case 1:
            xSemaphoreTake(mutex, portMAX_DELAY);
            usprintf(buf, "Count: %4d", led_counter);
            xSemaphoreGive(mutex);
            oled_disp_str_at(buf, 2, 0);

            usprintf(buf, "Button: %3d", button_counter);
            oled_disp_str_at(buf, 3, 0);

            oled_set_pos(6, 0);
            state = led_counter % 16;
            for (i = 0; i < state; ++i)
                oled_disp_char('_');
            oled_disp_char('*');
            for (i = state + 1; i < 16; ++i)
                oled_disp_char('_');

            oled_set_pos(7, 0);
            state = button_counter % 16;
            for (i = 0; i < state; ++i)
                oled_disp_char('_');
            oled_disp_char('*');
            for (i = state + 1; i < 16; ++i)
                oled_disp_char('_');
            break;

        case 2:
            dist = ultrasonic_get_distance();
            usprintf(buf, "Dist: %7d mm", (int)(dist * 10));
            oled_disp_str_at(buf, 2, 0);

//            if (button_counter % 2) {
//                buzzer_set_freq((int)(dist * 10 / 2));
//            } else {
//                buzzer_set_freq(0);
//            }

            battery = (float)adc_get_value() * 3.3 / 4095.0 * 78.0 / 10.0;
            usprintf(buf, "Battery: %2d.%1d V", (int)battery, (int)(battery * 10) % 10);
            oled_disp_str_at(buf, 3, 0);

            imu_poll_sensors(&sensors);
            imu_sensors_transform(&sensors, &real_sens);
            imu_estimate_triad(real_sens.acc, real_sens.mag, &angles);

            usprintf(buf, "P: %4d", (int)angles.y);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "R: %4d", (int)angles.x);
            oled_disp_str_at(buf, 6, 0);
            usprintf(buf, "Y: %4d", (int)angles.z);
            oled_disp_str_at(buf, 7, 0);
            break;

        case 3:
            adxl345_get_accel(&sensors.acc.x, &sensors.acc.y, &sensors.acc.z);
            oled_disp_str_at("Accelerometer", 2, 0);
            usprintf(buf, "X: %5d", sensors.acc.x);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Y: %5d", sensors.acc.y);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "Z: %5d", sensors.acc.z);
            oled_disp_str_at(buf, 6, 0);
            break;

        case 4:
            l3g4200d_read_gyro(&sensors.gyro.x, &sensors.gyro.y, &sensors.gyro.z);
            oled_disp_str_at("Gyroscope", 2, 0);
            usprintf(buf, "X: %5d", sensors.gyro.x);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Y: %5d", sensors.gyro.y);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "Z: %5d", sensors.gyro.z);
            oled_disp_str_at(buf, 6, 0);
            break;

        case 5:
            hmc5883_read_mag(&sensors.mag.x, &sensors.mag.y, &sensors.mag.z);
            oled_disp_str_at("Magnetometer", 2, 0);
            usprintf(buf, "X: %5d", sensors.mag.x);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Y: %5d", sensors.mag.y);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "Z: %5d", sensors.mag.z);
            oled_disp_str_at(buf, 6, 0);
            break;

        case 6:
            sensors.temperature = bmp085_read_temp();
            sensors.pressure = bmp085_read_pressure();
            oled_disp_str_at("Barometer", 2, 0);
            usprintf(buf, "Temp: %7d", sensors.temperature);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Pres: %7d", sensors.pressure);
            oled_disp_str_at(buf, 5, 0);
            break;

        case 7:
            oled_disp_str_at("GPS NMEA", 2, 0);

            if (gps_get_message(&msg))
            {
                oled_clear_rect(3, 0, 128, 5);
                oled_disp_str_at((char*)msg.command, 3, 0);
                oled_disp_str_at((char*)msg.data, 4, 0);
            }
            break;

        default:
            break;
        }

        DELAY_MS(100);
    }
}


