#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"

#include "nrf24/nrf24.h"
#include "servo/servo.h"
#include "imu/i2c.h"
#include "imu/adxl345.h"
#include "imu/l3g4200d.h"
#include "imu/hmc5883.h"
#include "imu/bmp085.h"

#define LED_GPIO_BASE   GPIO_PORTF_BASE
#define LED_RED         GPIO_PIN_1
#define LED_GREEN       GPIO_PIN_3
#define LED_BLUE        GPIO_PIN_2

void main(void)
{
    unsigned long i;
    uint8_t len;
    uint8_t recv[15] = "0123456789abcd", addr[6];
    uint8_t buffer[100];

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    SysTickPeriodSet(SysCtlClockGet() / 1000000UL);

    IntMasterEnable();
    SysTickIntEnable();

    SysTickEnable();

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 9600,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    GPIOPinTypeGPIOOutput(LED_GPIO_BASE, LED_RED | LED_GREEN | LED_BLUE);
    GPIOPinWrite(LED_GPIO_BASE, LED_RED | LED_GREEN | LED_BLUE, 0x00);

    nRF24_config();

    servoConfig();
    servoInit();

    servoSetPulse(SERVO0_BASE, SERVO0_TIMER, 1000);
    servoSetPulse(SERVO1_BASE, SERVO1_TIMER, 1000);
    servoSetPulse(SERVO2_BASE, SERVO2_TIMER, 1000);
    servoSetPulse(SERVO3_BASE, SERVO3_TIMER, 1000);

    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0xFF);
    nRF24_delay(100000UL);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0xFF);
    nRF24_delay(100000UL);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0xFF);
    nRF24_delay(100000UL);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0xFF);
    nRF24_delay(100000UL);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0xFF);
    nRF24_delay(100000UL);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
    nRF24_delay(100000UL);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0xFF);
    nRF24_delay(100000UL);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
    nRF24_delay(100000UL);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0xFF);
    nRF24_delay(100000UL);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);

    // I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(), false);

    UARTStdioInit(0);

    UARTprintf("*** Quadcopter Test ***\r\n");

    nRF24_init();
    nRF24_setChannel(23);
    nRF24_setPayloadSize(15);
    nRF24_setRF(NRF24DataRate2Mbps, NRF24TransmitPower0dBm);

    ADXL345_setOffsets(0, 0, 0);
    ADXL345_init();
    L3G4200D_init();
    HMC5883_init();
    BMP085_init();

    int16_t ax, ay, az, gx, gy, gz, mx, my, mz, temp;
    int32_t pressure;

    while (1)
    {
        ADXL345_getAcceleration(&ax, &ay, &az);
        L3G4200D_readGyro(&gx, &gy, &gz);
        HMC5883_readMag(&mx, &my, &mz);
        temp = BMP085_readTemperature();
        pressure = BMP085_readPressure();
//        UARTprintf("aX: %4d, aY: %4d, aZ: %4d | gX: %6d, gY: %6d, gZ: %6d\r\n",
//                   ax, ay, az, gx, gy, gz);
//        UARTprintf("mX: %6d, mY: %6d, mZ: %6d\r\n", mx, my, mz);
        UARTprintf("temp: %6d, press: %6d\r\n", temp, pressure);
        nRF24_delay(10000UL);
    }

//    while (1)
//    {
//        char c = UARTCharGet(UART2_BASE);
//        UARTCharPut(UART0_BASE, c);
//    }

    while (1)
    {
//        UARTprintf("Waiting for available data...\r\n");

//        nRF24_waitAvailable();
//        if (!nRF24_receive(recv, &len))
//            UARTprintf("Receive failed!\r\n");
//        else
//            UARTprintf("Received: %s\r\n", recv);

//        GPIOPinWrite(LED_GPIO_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_BLUE);
//
//        UARTprintf("Transmitting %d\r\n", temp);
//
//        addr[5] = '\0';
//        nRF24_readRegisterBurst(NRF24_REG_0A_RX_ADDR_P0, addr, 5);
//        UARTprintf("RX %s\r\n", addr);
//        nRF24_readRegisterBurst(NRF24_REG_10_TX_ADDR, addr, 5);
//        UARTprintf("TX %s\r\n", addr);
//        UARTprintf("RW %d\r\n", nRF24_readRegister(NRF24_REG_11_RX_PW_P0));
//
//        nRF24_setTxAddress((uint8_t*) "clie1");
//        nRF24_send(recv, 15, false);
//        if (!nRF24_waitPacketSent())
//        {
//            GPIOPinWrite(LED_GPIO_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_RED);
//
//            UARTprintf("waitPacketSent failed! %d\r\n",
//                    nRF24_readRegister(NRF24_REG_08_OBSERVE_TX) >> 4);
//
//            nRF24_delay(1000000UL);
//            continue;
//        }
//        else
//            UARTprintf("SENT\r\n");
//
//        GPIOPinWrite(LED_GPIO_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_GREEN);

        GPIOPinWrite(LED_GPIO_BASE, LED_RED | LED_GREEN | LED_BLUE, 0x00);
        UARTprintf("Receiving...\r\n");

        nRF24_setRxAddress((uint8_t*) "serv1");
        nRF24_readRegisterBurst(NRF24_REG_0A_RX_ADDR_P0, addr, 5);
        UARTprintf("RX %s\r\n", addr);

        nRF24_waitAvailable();
//        if (!nRF24_waitAvailableTimeout(1000000UL))
//        {
//            GPIOPinWrite(LED_GPIO_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_RED);
//
//            UARTprintf("waitAvailableTimeout failed!\r\n");
//
//            //nRF24_delay(1000000UL);
//            continue;
//        }
        nRF24_receive(recv, &len);

        servoSetPulse(SERVO0_BASE, SERVO0_TIMER, (recv[0] << 8) | recv[1]);
        servoSetPulse(SERVO1_BASE, SERVO1_TIMER, (recv[2] << 8) | recv[3]);
        servoSetPulse(SERVO2_BASE, SERVO2_TIMER, (recv[4] << 8) | recv[5]);
        servoSetPulse(SERVO3_BASE, SERVO3_TIMER, (recv[6] << 8) | recv[7]);

        UARTprintf("Received: %s\r\n", recv);

        //UARTprintf("status 0x%02x\r\n", nRF24_getStatus());

        GPIOPinWrite(LED_GPIO_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_GREEN);

        //nRF24_delay(1000000UL);
    }
}
