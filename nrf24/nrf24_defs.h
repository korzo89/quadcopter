/*
Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#ifndef NRF24_DEFS_H_
#define NRF24_DEFS_H_

// This is the bit in the SPI address that marks it as a write
#define NRF24_SPI_WRITE_MASK 0x80

// This is the maximum message length that can be supported by this library. Limited by
// the suported message lengths oin the nRF24
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
#ifndef NRF24_MAX_MESSAGE_LEN
#define NRF24_MAX_MESSAGE_LEN 32
#endif

// Keep track of the mode the NRF24 is in
#define NRF24_MODE_IDLE         0
#define NRF24_MODE_RX           1
#define NRF24_MODE_TX           2

// These values we set for FIFO thresholds are actually the same as the POR values
#define NRF24_TXFFAEM_THRESHOLD 4
#define NRF24_RXFFAFULL_THRESHOLD 55

// This is the default node address,
#define NRF24_DEFAULT_NODE_ADDRESS 0x00000000

// This address in the TO addreess signifies a broadcast
#define NRF24_BROADCAST_ADDRESS 0xffffffffff

// SPI CMD names
#define NRF24_CMD_R_REGISTER                        0x00
#define NRF24_CMD_W_REGISTER                        0x20
#define NRF24_CMD_R_RX_PAYLOAD                      0x61
#define NRF24_CMD_W_TX_PAYLOAD                      0xa0
#define NRF24_CMD_FLUSH_TX                          0xe1
#define NRF24_CMD_FLUSH_RX                          0xe2
#define NRF24_CMD_REUSE_TX_PL                       0xe3
#define NRF24_CMD_R_RX_PL_WID                       0x60
#define NRF24_CMD_W_ACK_PAYLOAD(pipe)               (0xa8|(pipe&0x7))
#define NRF24_CMD_W_TX_PAYLOAD_NOACK                0xb0
#define NRF24_CMD_NOP                               0xff

// Register names
#define NRF24_REGISTER_MASK                             0x1f
#define NRF24_REG_00_CONFIG                             0x00
#define NRF24_REG_01_EN_AA                              0x01
#define NRF24_REG_02_EN_RXADDR                          0x02
#define NRF24_REG_03_SETUP_AW                           0x03
#define NRF24_REG_04_SETUP_RETR                         0x04
#define NRF24_REG_05_RF_CH                              0x05
#define NRF24_REG_06_RF_SETUP                           0x06
#define NRF24_REG_07_STATUS                             0x07
#define NRF24_REG_08_OBSERVE_TX                         0x08
#define NRF24_REG_09_RPD                                0x09
#define NRF24_REG_0A_RX_ADDR_P0                         0x0a
#define NRF24_REG_0B_RX_ADDR_P1                         0x0b
#define NRF24_REG_0C_RX_ADDR_P2                         0x0c
#define NRF24_REG_0D_RX_ADDR_P3                         0x0d
#define NRF24_REG_0E_RX_ADDR_P4                         0x0e
#define NRF24_REG_0F_RX_ADDR_P5                         0x0f
#define NRF24_REG_10_TX_ADDR                            0x10
#define NRF24_REG_11_RX_PW_P0                           0x11
#define NRF24_REG_12_RX_PW_P1                           0x12
#define NRF24_REG_13_RX_PW_P2                           0x13
#define NRF24_REG_14_RX_PW_P3                           0x14
#define NRF24_REG_15_RX_PW_P4                           0x15
#define NRF24_REG_16_RX_PW_P5                           0x16
#define NRF24_REG_17_FIFO_STATUS                        0x17
#define NRF24_REG_1C_DYNPD                              0x1c
#define NRF24_REG_1D_FEATURE                            0x1d

// These register masks etc are named wherever possible
// corresponding to the bit and field names in the nRF24L01 Product Specification
// #define NRF24_REG_00_CONFIG                             0x00
#define NRF24_MASK_RX_DR                                0x40
#define NRF24_MASK_TX_DS                                0x20
#define NRF24_MASK_MAX_RT                               0x10
#define NRF24_EN_CRC                                    0x08
#define NRF24_CRCO                                      0x04
#define NRF24_PWR_UP                                    0x02
#define NRF24_PRIM_RX                                   0x01

// #define NRF24_REG_01_EN_AA                              0x01
#define NRF24_ENAA_P5                                   0x20
#define NRF24_ENAA_P4                                   0x10
#define NRF24_ENAA_P3                                   0x08
#define NRF24_ENAA_P2                                   0x04
#define NRF24_ENAA_P1                                   0x02
#define NRF24_ENAA_P0                                   0x01

// #define NRF24_REG_02_EN_RXADDR                          0x02
#define NRF24_ERX_P5                                    0x20
#define NRF24_ERX_P4                                    0x10
#define NRF24_ERX_P3                                    0x08
#define NRF24_ERX_P2                                    0x04
#define NRF24_ERX_P1                                    0x02
#define NRF24_ERX_P0                                    0x01

// #define NRF24_REG_03_SETUP_AW                           0x03
#define NRF24_AW_3_BYTES                                0x01
#define NRF24_AW_4_BYTES                                0x02
#define NRF24_AW_5_BYTES                                0x03

// #define NRF24_REG_04_SETUP_RETR                         0x04
#define NRF24_ARD                                       0xf0
#define NRF24_ARC                                       0x0f

// #define NRF24_REG_05_RF_CH                              0x05
#define NRF24_RF_CH                                     0x7f

// #define NRF24_REG_06_RF_SETUP                           0x06
#define NRF24_CONT_WAVE                                 0x80
#define NRF24_RF_DR_LOW                                 0x20
#define NRF24_PLL_LOCK                                  0x10
#define NRF24_RF_DR_HIGH                                0x08
#define NRF24_PWR                                       0x06
#define NRF24_PWR_m18dBm                                0x00
#define NRF24_PWR_m12dBm                                0x02
#define NRF24_PWR_m6dBm                                 0x04
#define NRF24_PWR_0dBm                                  0x06

// #define NRF24_REG_07_STATUS                             0x07
#define NRF24_RX_DR                                     0x40
#define NRF24_TX_DS                                     0x20
#define NRF24_MAX_RT                                    0x10
#define NRF24_RX_P_NO                                   0x0e
#define NRF24_STATUS_TX_FULL                            0x01


// #define NRF24_REG_08_OBSERVE_TX                         0x08
#define NRF24_PLOS_CNT                                  0xf0
#define NRF24_ARC_CNT                                   0x0f

// #define NRF24_REG_09_RPD                                0x09
#define NRF24_RPD                                       0x01

// #define NRF24_REG_17_FIFO_STATUS                        0x17
#define NRF24_TX_REUSE                                  0x40
#define NRF24_TX_FULL                                   0x20
#define NRF24_TX_EMPTY                                  0x10
#define NRF24_RX_FULL                                   0x02
#define NRF24_RX_EMPTY                                  0x01

// #define NRF24_REG_1C_DYNPD                              0x1c
#define NRF24_DPL_P5                                    0x20
#define NRF24_DPL_P4                                    0x10
#define NRF24_DPL_P3                                    0x08
#define NRF24_DPL_P2                                    0x04
#define NRF24_DPL_P1                                    0x02
#define NRF24_DPL_P0                                    0x01

// #define NRF24_REG_1D_FEATURE                            0x1d
#define NRF24_EN_DPL                                    0x04
#define NRF24_EN_ACK_PAY                                0x02
#define NRF24_EN_DYN_ACK                                0x01

#define NRF24_DUMMY_BYTE    0xFF

#define NRF24_PIN_HIGH      0xFF
#define NRF24_PIN_LOW       0x00

#endif /* NRF24_DEFS_H_ */
