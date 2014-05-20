/*
 * nrf24l01.h
 *
 *  Created on: 29-10-2013
 *      Author: Korzo
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

#include "nrf24l01_defs.h"

//-----------------------------------------------------------------

#define NRF_CHECK_STATUS(x)     ( nrf_get_status() & (x) )
#define NRF_CHECK_FIFO(x)		( nrf_get_fifo_status() & (x) )

//-----------------------------------------------------------------

typedef void (*irq_callback_t)(void);

//-----------------------------------------------------------------

void nrf_init(void);

uint8_t nrf_exec_cmd(uint8_t cmd, uint8_t *data, unsigned int len, bool read);

uint8_t nrf_write_register(uint8_t reg, uint8_t *data, unsigned int len);
uint8_t nrf_write_register_byte(uint8_t reg, uint8_t data);

uint8_t nrf_read_register(uint8_t reg, uint8_t *data, unsigned int len);
uint8_t nrf_read_register_byte(uint8_t reg);

void nrf_power_up(void);

uint8_t nrf_write_tx_payload(uint8_t *data, unsigned int len, bool transmit);
uint8_t nrf_read_rx_payload(uint8_t *data, unsigned int len);
uint8_t nrf_flush_tx(void);
uint8_t nrf_flush_rx(void);
uint8_t nrf_nop(void);

void nrf_set_as_rx(void);
void nrf_set_as_tx(void);

void nrf_set_config(uint8_t config);
void nrf_set_rf_channel(uint8_t channel);
uint8_t nrf_get_status(void);
uint8_t nrfGetObserveTx(void);
void nrf_set_rx_addr(uint8_t *addr, unsigned int len, uint8_t pipe);
void nrf_set_tx_addr(uint8_t *addr, unsigned int len);
void nrf_set_payload_width(uint8_t width, uint8_t pipe);
uint8_t nrf_get_fifo_status(void);

void nrf_auto_ack_enable(uint8_t pipe);
void nrf_pipe_nable(uint8_t pipe);

bool nrf_carrier_detect(void);
uint8_t nrf_get_rx_pipe(void);

void nrf_clear_irq(uint8_t irq);
void nrf_clear_all_irq(void);
void nrf_clear_flush(void);

void nrf_transmit(void);

bool nrf_is_irq_active(void);
void nrf_set_irq_callback(irq_callback_t callback);

//-----------------------------------------------------------------

#endif /* NRF24L01_H_ */
