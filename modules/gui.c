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
#include <utils/buzzer_seq.h>

//-----------------------------------------------------------------

static xQueueHandle button_queue;

static char buf[17];

//-----------------------------------------------------------------

static void gui_task(void *params);
static void button_task(void *params);

//-----------------------------------------------------------------

result_t gui_init(void)
{
    button_queue = xQueueCreate(10, sizeof(uint8_t));

    if (xTaskCreate(button_task, TASK_NAME("BTN"),
            configMINIMAL_STACK_SIZE, NULL, 1, NULL) != pdPASS)
        return RES_ERR_FATAL;

    if (xTaskCreate(gui_task, TASK_NAME("GUI"),
            GUI_TASK_STACK, NULL, GUI_TASK_PRIORITY, NULL) != pdPASS)
        return RES_ERR_FATAL;

    return RES_OK;
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

static void gui_disp_control(const char *name, uint8_t row, char mode, float val)
{
    int dec = (int)val;
    int frac = abs((int)(val * 10) % 10);

    usprintf(buf, "%5s %c %6d.%1d", name, mode, dec, frac);
    oled_disp_str_at(buf, row, 0);
}

//-----------------------------------------------------------------

static void gui_disp_control_axis(const char *name, uint8_t row, const struct control_axis_val *axis)
{
    char mode;
    switch (axis->mode)
    {
    case AXIS_MODE_ANGLE:
        mode = 'A';
        break;
    case AXIS_MODE_RATE:
        mode = 'R';
        break;
    default:
        mode = 'D';
        break;
    }

    gui_disp_control(name, row, mode, axis->value);
}

//-----------------------------------------------------------------

static void gui_disp_motor(uint8_t num, uint8_t row, float val)
{
    usprintf(buf, "M%d %4d", num, (int)val);
    oled_disp_str_at(buf, row, 0);
}

//-----------------------------------------------------------------

const uint8_t SYMB_CONN[] = {
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

    uint8_t state;
    float dist;
    float battery;

    static struct gps_msg msg;
    static struct imu_sensor_data sensors;
    static struct imu_real real_sens;
    static struct vec3 triad, angles;

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
                screen = (screen + 1) % NUM_SCREENS;

                oled_clear();

                buzzer_seq_lib_play(BUZZER_SEQ_PRESS, BUZZER_MODE_IGNORE);
                break;
            case 2:
                break;
            default:
                break;
            }
        }

        oled_set_pos(0, 0);
        oled_disp_symbol((uint8_t*)(rcp_is_connected() ? SYMB_CONN : SYMB_LOST), 16);
        usprintf(buf, "    %d/%d    ", screen + 1, NUM_SCREENS);
        oled_disp_str(buf);
        oled_disp_str(control_is_armed() ? "ARM" : "   ");

        switch (screen)
        {
        case 0:
        {
            struct control_vals control;
            control_get_vals(&control);

            gui_disp_control("1 Thr", 2, 'T', control.throttle);
            gui_disp_control_axis("2 Pit", 3, &control.pitch);
            gui_disp_control_axis("3 Rol", 4, &control.roll);
            gui_disp_control_axis("4 Yaw", 5, &control.yaw);
            oled_set_pos(6, 0);
            oled_disp_str(control_is_armed() ? "ARMED   " : "DISARMED");

            break;
        }

        case 1:
        {
            struct control_motors motors;
            control_get_motors(&motors);

            /***0123456789ABCDEF**/
            /*********************/
            /*0 *Y))   2/8    ARM*/
            /*1 *                */
            /*2 *M1 1000  M3   M2*/
            /*3 *M2 1000    \ /  */
            /*4 *M3 1000     O   */
            /*5 *M4 1000    / \  */
            /*6 *         M4 | M1*/
            /*7 *            V   */
            /*********************/

            gui_disp_motor(1, 2, motors.m1);
            gui_disp_motor(2, 3, motors.m2);
            gui_disp_motor(3, 4, motors.m3);
            gui_disp_motor(4, 5, motors.m4);

            oled_disp_str_at("M3   M2", 2, 9);
            oled_disp_str_at( "\\ /",   3, 11);
            oled_disp_str_at(   "O",    4, 12);
            oled_disp_str_at(  "/ \\",  5, 11);
            oled_disp_str_at("M4 | M1", 6, 9);
            oled_disp_str_at(   "V",    7, 12);

            break;
        }

        case 2:
            dist = ultrasonic_get_distance();
            usprintf(buf, "Dist: %7d mm", (int)(dist * 10));
            oled_disp_str_at(buf, 2, 0);

            battery = (float)adc_get_value() * 3.3 / 4095.0 * 78.0 / 10.0;
            usprintf(buf, "Battery: %2d.%1d V", (int)battery, (int)(battery * 10) % 10);
            oled_disp_str_at(buf, 3, 0);

            imu_get_sensors(&sensors);
            imu_sensors_transform(&sensors, &real_sens);
            imu_estimate_triad(real_sens.acc, real_sens.mag, NULL, &triad);
            imu_get_angles(&angles);

            oled_disp_str_at("   MADGW  TRIAD", 4, 0);
            usprintf(buf, "P: %5d  %5d", (int)angles.y, (int)triad.y);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "R: %5d  %5d", (int)angles.x, (int)triad.x);
            oled_disp_str_at(buf, 6, 0);
            usprintf(buf, "Y: %5d  %5d", (int)angles.z, (int)triad.z);
            oled_disp_str_at(buf, 7, 0);
            break;

        case 3:
            imu_get_sensors(&sensors);
            oled_disp_str_at("Accelerometer", 2, 0);
            usprintf(buf, "X: %5d", sensors.acc.x);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Y: %5d", sensors.acc.y);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "Z: %5d", sensors.acc.z);
            oled_disp_str_at(buf, 6, 0);
            break;

        case 4:
            imu_get_sensors(&sensors);
            oled_disp_str_at("Gyroscope", 2, 0);
            usprintf(buf, "X: %5d", sensors.gyro.x);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Y: %5d", sensors.gyro.y);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "Z: %5d", sensors.gyro.z);
            oled_disp_str_at(buf, 6, 0);
            break;

        case 5:
            imu_get_sensors(&sensors);
            oled_disp_str_at("Magnetometer", 2, 0);
            usprintf(buf, "X: %5d", sensors.mag.x);
            oled_disp_str_at(buf, 4, 0);
            usprintf(buf, "Y: %5d", sensors.mag.y);
            oled_disp_str_at(buf, 5, 0);
            usprintf(buf, "Z: %5d", sensors.mag.z);
            oled_disp_str_at(buf, 6, 0);
            break;

        case 6:
            imu_get_sensors(&sensors);
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


