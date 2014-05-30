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

#include <modules/imu.h>
#include <modules/rcp.h>
#include <modules/gui.h>
#include <modules/gps.h>
#include <modules/control.h>

//-----------------------------------------------------------------

#define SYSTEM_INIT_TASK_STACK  256

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
    oled_init();
    buzzer_init();
    led_init();

    motors_init();
    motors_set_throttle(0, 0, 0, 0);

    if (BUTTON_PRESSED())
    {
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

        uint8_t eep[4] = { 0 };
        eeprom_read(0x0000, eep, 4);

        usprintf(buf, "%02x %02x %02x %02x", eep[0], eep[1], eep[2], eep[3]);
        oled_disp_str(buf);

        while (BUTTON_PRESSED());
        while (!BUTTON_PRESSED());
        while (BUTTON_PRESSED());

        led_set(LEDS_OFF);
    }

    adc_init();
    ultrasonic_init();
    imu_init(0.1f, 100.0f);

    rcp_init();
    gps_init();
    control_init();

    gui_init();

    vTaskDelete(NULL);
}

//-----------------------------------------------------------------

result_t system_init(void)
{
    system_hw_init();

    if (xTaskCreate(system_init_task, (signed portCHAR*)"INIT",
            SYSTEM_INIT_TASK_STACK, NULL, 1, NULL) != pdPASS)
        return RES_ERR_FATAL;

    vTaskStartScheduler();

    return RES_OK;
}


