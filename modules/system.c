/*
 * system.c
 *
 *  Created on: 30 maj 2014
 *      Author: Korzo
 */

#include "system.h"

#include <stellaris_config.h>

#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <portmacro.h>
#include <task.h>
#include <semphr.h>

#include <drivers/ext_i2c.h>
#include <drivers/oled.h>
#include <drivers/eeprom.h>
#include <drivers/buzzer.h>
#include <drivers/led.h>
#include <drivers/motors.h>
#include <drivers/ultrasonic.h>
#include <drivers/adc.h>

#include <modules/imu.h>
#include <modules/rcp.h>
#include <modules/gui.h>
#include <modules/gps.h>
#include <modules/control.h>
#include <modules/params.h>
#include <modules/daq.h>

#include <utils/ustdlib.h>
#include <utils/uartstdio.h>
#include <utils/delay.h>
#include <utils/buzzer_notes.h>

//-----------------------------------------------------------------

static void system_init_task(void *params);
static void system_hw_init(void);

static void esc_calibration(void);

//-----------------------------------------------------------------

result_t system_init(void)
{
    system_hw_init();

    if (xTaskCreate(system_init_task, TASK_NAME("INIT"),
            SYSTEM_INIT_TASK_STACK, NULL, SYSTEM_INIT_TASK_PRIORITY, NULL) != pdPASS)
        return RES_ERR_FATAL;

    vTaskStartScheduler();

    return RES_OK;
}

//-----------------------------------------------------------------

static void system_hw_init(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
            SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    IntMasterEnable();

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    UARTStdioInit(0);
}

//-----------------------------------------------------------------

static void system_init_task(void *params)
{
    ext_i2c_init();

    eeprom_init();
    buzzer_init();
    led_init();

    bool has_oled = oled_init();

    motors_init();

    if (BUTTON_PRESSED())
    {
#if 0
        char buf[17];

        led_set(LED_GREEN);
        buzzer_set_freq(NOTE_C5);
        DELAY_MS(100);
        led_set(LED_GREEN | LED_YELLOW);
        buzzer_set_freq(NOTE_D5);
        DELAY_MS(100);
        led_set(LED_GREEN | LED_YELLOW | LED_RED);
        buzzer_set_freq(NOTE_E5);
        DELAY_MS(100);
        buzzer_set_freq(0);

        oled_clear();
        oled_disp_str("EEPROM read\n");

        uint8_t eep[5] = { 0 };
        eeprom_read(0x0000, eep, 5);

        usprintf(buf, "%02x %02x %02x %02x %02x", eep[0], eep[1], eep[2], eep[3], eep[4]);
        oled_disp_str(buf);

        while (BUTTON_PRESSED());
        while (!BUTTON_PRESSED());
        while (BUTTON_PRESSED());

        led_set(LEDS_OFF);
#endif
        esc_calibration();
    }

    motors_set_throttle(0, 0, 0, 0);

    rcp_init();

    params_init();
#if 0
    if (params_eeprom_load() != RES_OK)
        params_load_defaults();
#endif

    daq_init();

    adc_init();
    ultrasonic_init();
    imu_init();

    gps_init();
    control_init();

    if (has_oled)
        gui_init();

    vTaskDelete(NULL);
}

//-----------------------------------------------------------------

static void esc_calibration(void)
{
    oled_clear();
    oled_disp_str("ESC Calibration\n\n");

    led_turn_on(LED_GREEN);
    oled_disp_str("Setting 100%  ");
    motors_set_throttle(THROTTLE_MAX, THROTTLE_MAX, THROTTLE_MAX, THROTTLE_MAX);
    DELAY_MS(2000);
    oled_disp_str("OK");

    led_turn_on(LED_YELLOW);
    oled_disp_str("Setting 0%    ");
    motors_set_throttle(0, 0, 0, 0);
    DELAY_MS(2000);
    oled_disp_str("OK\n");

    led_turn_on(LED_RED);
    oled_disp_str("Calibration OK\n");

    buzzer_set_freq(NOTE_C5);
    DELAY_MS(100);
    buzzer_set_freq(NOTE_D5);
    DELAY_MS(100);
    buzzer_set_freq(NOTE_E5);
    DELAY_MS(100);
    buzzer_set_freq(0);

    while (BUTTON_PRESSED());
    while (!BUTTON_PRESSED());
    while (BUTTON_PRESSED());

    led_set(LEDS_OFF);
    oled_clear();
}


