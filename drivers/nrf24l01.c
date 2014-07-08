/*
 * nrf24l01.c
 *
 *  Created on: 29-10-2013
 *      Author: Korzo
 */

#include "nrf24l01.h"

#include <utils/delay.h>

#include <stellaris_config.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stdlib.h>

//-----------------------------------------------------------------

#define NRF_CE_PORT           GPIO_PORTA_BASE
#define NRF_CE_PIN            GPIO_PIN_3

#define NRF_CSN_PORT          GPIO_PORTA_BASE
#define NRF_CSN_PIN           GPIO_PIN_7

#define NRF_IRQ_PORT          GPIO_PORTC_BASE
#define NRF_IRQ_PIN           GPIO_PIN_7

#define NRF_SPI               SSI0_BASE

#define CE_SET()              GPIOPinWrite(NRF_CE_PORT, NRF_CE_PIN, 0xFF)
#define CE_CLEAR()            GPIOPinWrite(NRF_CE_PORT, NRF_CE_PIN, 0x00)

#define CSN_SET()             GPIOPinWrite(NRF_CSN_PORT, NRF_CSN_PIN, 0xFF)
#define CSN_CLEAR()           GPIOPinWrite(NRF_CSN_PORT, NRF_CSN_PIN, 0x00)

#define READ_IRQ_PIN()        GPIOPinRead(NRF_IRQ_PORT, NRF_IRQ_PIN)

#define NRF_DELAY_MS(x)       DELAY_MS(x)

//-----------------------------------------------------------------

static irq_callback_t nrf_irq_callback = NULL;

//-----------------------------------------------------------------

void GPIOCIntHandler(void)
{
    unsigned long status = GPIOPinIntStatus(NRF_IRQ_PORT, true);
    GPIOPinIntClear(NRF_IRQ_PORT, status);

    if ((status & NRF_IRQ_PIN) && nrf_irq_callback)
        nrf_irq_callback();
}

//-----------------------------------------------------------------

void nrf_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    // config nRF pins
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_7);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);

    CSN_SET();
    CE_CLEAR();

    // config IRQ pin
    GPIOPinTypeGPIOInput(NRF_IRQ_PORT, NRF_IRQ_PIN);
    GPIOIntTypeSet(NRF_IRQ_PORT, NRF_IRQ_PIN, GPIO_FALLING_EDGE);
    GPIOPinIntEnable(NRF_IRQ_PORT, NRF_IRQ_PIN);

    IntPrioritySet(INT_GPIOC, configKERNEL_INTERRUPT_PRIORITY);
    IntEnable(INT_GPIOC);

    // config SPI
    SSIConfigSetExpClk(NRF_SPI, SysCtlClockGet(),
                       SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(NRF_SPI);
}

//-----------------------------------------------------------------

static uint8_t nrf_spi_send_byte(uint8_t data)
{
    unsigned long result;

    SSIDataPut(NRF_SPI, data);
    while (SSIBusy(NRF_SPI));
    SSIDataGet(NRF_SPI, &result);

    return (uint8_t)result;
}

//-----------------------------------------------------------------

uint8_t nrf_exec_cmd(uint8_t cmd, uint8_t *data, unsigned int len, bool read)
{
    unsigned int i;
    uint8_t status, temp;

    CSN_CLEAR();

    status = nrf_spi_send_byte(cmd);
    for (i = 0; i < len; i++)
    {
        temp = nrf_spi_send_byte(data[i]);
        if (read)
            data[i] = temp;
    }

    CSN_SET();

    return status;
}

//-----------------------------------------------------------------

uint8_t nrf_write_register(uint8_t reg, uint8_t *data, unsigned int len)
{
    return nrf_exec_cmd(NRF_W_REGISTER | (reg & NRF_W_REGISTER_MASK), data, len, false);
}

//-----------------------------------------------------------------

uint8_t nrf_write_register_byte(uint8_t reg, uint8_t data)
{
    return nrf_write_register(reg, &data, 1);
}

//-----------------------------------------------------------------

uint8_t nrf_read_register(uint8_t reg, uint8_t *data, unsigned int len)
{
    return nrf_exec_cmd(NRF_R_REGISTER | (reg & NRF_R_REGISTER_MASK), data, len, true);
}

//-----------------------------------------------------------------

uint8_t nrf_read_register_byte(uint8_t reg)
{
    uint8_t res;
    nrf_read_register(reg, &res, 1);
    return res;
}

//-----------------------------------------------------------------

void nrf_power_up(void)
{
    uint8_t config = nrf_read_register_byte(NRF_CONFIG);
    if (config & NRF_CONFIG_PWR_UP)
        return;

    config |= NRF_CONFIG_PWR_UP;
    nrf_write_register_byte(NRF_CONFIG, config);
}

//-----------------------------------------------------------------

uint8_t nrf_write_tx_payload(uint8_t *data, unsigned int len, bool transmit)
{
    uint8_t status = nrf_exec_cmd(NRF_W_TX_PAYLOAD, data, len, false);

    if (transmit)
        nrf_transmit();

    return status;
}

//-----------------------------------------------------------------

uint8_t nrf_read_rx_payload(uint8_t *data, unsigned int len)
{
    uint8_t status;

    CE_CLEAR();
    status = nrf_exec_cmd(NRF_R_RX_PAYLOAD, data, len, true);
    CE_SET();

    return status;
}

//-----------------------------------------------------------------

uint8_t nrf_flush_tx(void)
{
    return nrf_exec_cmd(NRF_FLUSH_TX, NULL, 0, false);
}

//-----------------------------------------------------------------

uint8_t nrf_flush_rx(void)
{
    return nrf_exec_cmd(NRF_FLUSH_RX, NULL, 0, false);
}

//-----------------------------------------------------------------

uint8_t nrf_nop(void)
{
    return nrf_exec_cmd(NRF_NOP, NULL, 0, false);
}

//-----------------------------------------------------------------

void nrf_set_as_rx(void)
{
    CE_SET();

    uint8_t config = nrf_read_register_byte(NRF_CONFIG);
    if (config & NRF_CONFIG_PRIM_RX)
        return;

    config |= NRF_CONFIG_PRIM_RX;
    nrf_write_register_byte(NRF_CONFIG, config);
}

//-----------------------------------------------------------------

void nrf_set_as_tx(void)
{
    CE_CLEAR();

    uint8_t config = nrf_read_register_byte(NRF_CONFIG);
    if (!(config & NRF_CONFIG_PRIM_RX))
        return;

    config &= ~NRF_CONFIG_PRIM_RX;
    nrf_write_register_byte(NRF_CONFIG, config);
}

//-----------------------------------------------------------------

void nrf_set_config(uint8_t config)
{
    nrf_write_register_byte(NRF_CONFIG, config);
}

//-----------------------------------------------------------------

void nrf_set_rf_channel(uint8_t channel)
{
    channel &= ~NRF_RF_CH_RESERVED;
    nrf_write_register_byte(NRF_RF_CH, channel);
}

//-----------------------------------------------------------------

uint8_t nrf_get_status(void)
{
    return nrf_nop();
}

//-----------------------------------------------------------------

uint8_t nrf_get_observe_tx(void)
{
    return nrf_read_register_byte(NRF_OBSERVE_TX);
}

//-----------------------------------------------------------------

void nrf_set_rx_addr(uint8_t *addr, unsigned int len, uint8_t pipe)
{
    if (pipe > 5)
        return;

    nrf_write_register(NRF_RX_ADDR_P0 + pipe, addr, len);
}

//-----------------------------------------------------------------

void nrf_set_tx_addr(uint8_t *addr, unsigned int len)
{
    nrf_write_register(NRF_TX_ADDR, addr, len);
}

//-----------------------------------------------------------------

void nrf_set_payload_width(uint8_t width, uint8_t pipe)
{
    if (pipe > 5 || width > 32)
        return;

    nrf_write_register_byte(NRF_RX_PW_P0 + pipe, width);
}

//-----------------------------------------------------------------

uint8_t nrf_get_fifo_status(void)
{
    return nrf_read_register_byte(NRF_FIFO_STATUS);
}

//-----------------------------------------------------------------

void nrf_auto_ack_enable(uint8_t pipe)
{
    if (pipe > 5)
        return;

    uint8_t data = nrf_read_register_byte(NRF_EN_AA);
    if (data & (1 << pipe))
        return;

    data |= 1 << pipe;
    nrf_write_register_byte(NRF_EN_AA, data);
}

//-----------------------------------------------------------------

void nrf_pipe_enable(uint8_t pipe)
{
    if (pipe > 5)
        return;

    uint8_t data = nrf_read_register_byte(NRF_EN_RXADDR);
    if (data & (1 << pipe))
        return;

    data |= 1 << pipe;
    nrf_write_register_byte(NRF_EN_RXADDR, data);
}

//-----------------------------------------------------------------

bool nrf_carrier_detect(void)
{
    return nrf_read_register_byte(NRF_CD);
}

//-----------------------------------------------------------------

uint8_t nrf_get_rx_pipe(void)
{
    return (nrf_get_status() & NRF_STATUS_RX_P_NO) >> 1;
}

//-----------------------------------------------------------------

void nrf_clear_irq(uint8_t irq)
{
    nrf_write_register_byte(NRF_STATUS, irq);
}

//-----------------------------------------------------------------

void nrf_clear_all_irq(void)
{
    nrf_clear_irq(NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
}

//-----------------------------------------------------------------

void nrf_clear_flush(void)
{
    nrf_clear_all_irq();
    nrf_flush_rx();
    nrf_flush_tx();
}

//-----------------------------------------------------------------

void nrf_transmit(void)
{
    CE_SET();
    NRF_DELAY_MS(1);
    CE_CLEAR();
}

//-----------------------------------------------------------------

bool nrf_is_irq_active(void)
{
    return !READ_IRQ_PIN();
}

//-----------------------------------------------------------------

void nrf_set_irq_callback(irq_callback_t callback)
{
    nrf_irq_callback = callback;
}
