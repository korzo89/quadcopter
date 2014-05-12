/*
 * NRF_defs.h
 *
 *  Created on: 30-10-2013
 *      Author: Korzo
 */

#ifndef NRF24L01_DEFS_H_
#define NRF24L01_DEFS_H_

//-----------------------------------------------------------------

// SPI commands
#define NRF_R_REGISTER             0x00
#define NRF_W_REGISTER             0x20
#define NRF_R_RX_PAYLOAD           0x61
#define NRF_W_TX_PAYLOAD           0xA0
#define NRF_FLUSH_TX               0xE1
#define NRF_FLUSH_RX               0xE2
#define NRF_REUSE_TX_PL            0xE3
#define NRF_NOP                    0xFF

// SPI command masks
#define NRF_R_REGISTER_MASK        0x1F
#define NRF_W_REGISTER_MASK        0x1F

// registers
#define NRF_CONFIG                 0x00
#define NRF_EN_AA                  0x01
#define NRF_EN_RXADDR              0x02
#define NRF_SETUP_AW               0x03
#define NRF_SETUP_RETR             0x04
#define NRF_RF_CH                  0x05
#define NRF_RF_SETUP               0x06
#define NRF_STATUS                 0x07
#define NRF_OBSERVE_TX             0x08
#define NRF_CD                     0x09
#define NRF_RX_ADDR_P0             0x0A
#define NRF_RX_ADDR_P1             0x0B
#define NRF_RX_ADDR_P2             0x0C
#define NRF_RX_ADDR_P3             0x0D
#define NRF_RX_ADDR_P4             0x0E
#define NRF_RX_ADDR_P5             0x0F
#define NRF_TX_ADDR                0x10
#define NRF_RX_PW_P0               0x11
#define NRF_RX_PW_P1               0x12
#define NRF_RX_PW_P2               0x13
#define NRF_RX_PW_P3               0x14
#define NRF_RX_PW_P4               0x15
#define NRF_RX_PW_P5               0x16
#define NRF_FIFO_STATUS            0x17

// CONFIG register bits
#define NRF_CONFIG_RESERVED        0x80
#define NRF_CONFIG_MASK_RX_DR      0x40
#define NRF_CONFIG_MASK_TX_DS      0x20
#define NRF_CONFIG_MASK_MAX_RT     0x10
#define NRF_CONFIG_EN_CRC          0x08
#define NRF_CONFIG_CRCO            0x04
#define NRF_CONFIG_PWR_UP          0x02
#define NRF_CONFIG_PRIM_RX         0x01

// EN_AA register bits
#define NRF_EN_AA_RESERVED         0xC0
#define NRF_EN_AA_ENAA_ALL         0x3F
#define NRF_EN_AA_ENAA_P5          0x20
#define NRF_EN_AA_ENAA_P4          0x10
#define NRF_EN_AA_ENAA_P3          0x08
#define NRF_EN_AA_ENAA_P2          0x04
#define NRF_EN_AA_ENAA_P1          0x02
#define NRF_EN_AA_ENAA_P0          0x01
#define NRF_EN_AA_ENAA_NONE        0x00

// EN_RXADDR register bits
#define NRF_EN_RXADDR_RESERVED     0xC0
#define NRF_EN_RXADDR_ERX_ALL      0x3F
#define NRF_EN_RXADDR_ERX_P5       0x20
#define NRF_EN_RXADDR_ERX_P4       0x10
#define NRF_EN_RXADDR_ERX_P3       0x08
#define NRF_EN_RXADDR_ERX_P2       0x04
#define NRF_EN_RXADDR_ERX_P1       0x02
#define NRF_EN_RXADDR_ERX_P0       0x01
#define NRF_EN_RXADDR_ERX_NONE     0x00

// SETUP_AW register bits
#define NRF_SETUP_AW_RESERVED      0xFC
#define NRF_SETUP_AW               0x03
#define NRF_SETUP_AW_5BYTES        0x03
#define NRF_SETUP_AW_4BYTES        0x02
#define NRF_SETUP_AW_3BYTES        0x01
#define NRF_SETUP_AW_ILLEGAL       0x00

// SETUP_RETR register bits
#define NRF_SETUP_RETR_ARD         0xF0
#define NRF_SETUP_RETR_ARD_4000    0xF0
#define NRF_SETUP_RETR_ARD_3750    0xE0
#define NRF_SETUP_RETR_ARD_3500    0xD0
#define NRF_SETUP_RETR_ARD_3250    0xC0
#define NRF_SETUP_RETR_ARD_3000    0xB0
#define NRF_SETUP_RETR_ARD_2750    0xA0
#define NRF_SETUP_RETR_ARD_2500    0x90
#define NRF_SETUP_RETR_ARD_2250    0x80
#define NRF_SETUP_RETR_ARD_2000    0x70
#define NRF_SETUP_RETR_ARD_1750    0x60
#define NRF_SETUP_RETR_ARD_1500    0x50
#define NRF_SETUP_RETR_ARD_1250    0x40
#define NRF_SETUP_RETR_ARD_1000    0x30
#define NRF_SETUP_RETR_ARD_750     0x20
#define NRF_SETUP_RETR_ARD_500     0x10
#define NRF_SETUP_RETR_ARD_250     0x00
#define NRF_SETUP_RETR_ARC         0x0F
#define NRF_SETUP_RETR_ARC_15      0x0F
#define NRF_SETUP_RETR_ARC_14      0x0E
#define NRF_SETUP_RETR_ARC_13      0x0D
#define NRF_SETUP_RETR_ARC_12      0x0C
#define NRF_SETUP_RETR_ARC_11      0x0B
#define NRF_SETUP_RETR_ARC_10      0x0A
#define NRF_SETUP_RETR_ARC_9       0x09
#define NRF_SETUP_RETR_ARC_8       0x08
#define NRF_SETUP_RETR_ARC_7       0x07
#define NRF_SETUP_RETR_ARC_6       0x06
#define NRF_SETUP_RETR_ARC_5       0x05
#define NRF_SETUP_RETR_ARC_4       0x04
#define NRF_SETUP_RETR_ARC_3       0x03
#define NRF_SETUP_RETR_ARC_2       0x02
#define NRF_SETUP_RETR_ARC_1       0x01
#define NRF_SETUP_RETR_ARC_0       0x00

// RF_CH register bits
#define NRF_RF_CH_RESERVED 0x80

// RF_SETUP register bits
#define NRF_RF_SETUP_RESERVED      0xE0
#define NRF_RF_SETUP_PLL_LOCK      0x10
#define NRF_RF_SETUP_RF_DR         0x08
#define NRF_RF_SETUP_RF_PWR        0x06
#define NRF_RF_SETUP_RF_PWR_0      0x06
#define NRF_RF_SETUP_RF_PWR_6      0x04
#define NRF_RF_SETUP_RF_PWR_12     0x02
#define NRF_RF_SETUP_RF_PWR_18     0x00
#define NRF_RF_SETUP_LNA_HCURR     0x01

// STATUS register bits
#define NRF_STATUS_RESERVED                   0x80
#define NRF_STATUS_RX_DR                      0x40
#define NRF_STATUS_TX_DS                      0x20
#define NRF_STATUS_MAX_RT                     0x10
#define NRF_STATUS_RX_P_NO                    0x0E
#define NRF_STATUS_RX_P_NO_RX_FIFO_NOT_EMPTY  0x0E
#define NRF_STATUS_RX_P_NO_UNUSED             0x0C
#define NRF_STATUS_RX_P_NO_5                  0x0A
#define NRF_STATUS_RX_P_NO_4                  0x08
#define NRF_STATUS_RX_P_NO_3                  0x06
#define NRF_STATUS_RX_P_NO_2                  0x04
#define NRF_STATUS_RX_P_NO_1                  0x02
#define NRF_STATUS_RX_P_NO_0                  0x00
#define NRF_STATUS_TX_FULL                    0x01

// OBSERVE_TX register bits
#define NRF_OBSERVE_TX_PLOS_CNT    0xF0
#define NRF_OBSERVE_TX_ARC_CNT     0x0F

// CD register bits
#define NRF_CD_RESERVED            0xFE
#define NRF_CD_CD                  0x01

// RX_PW_P0 register bits
#define NRF_RX_PW_P0_RESERVED      0xC0

// RX_PW_P0 register bits
#define NRF_RX_PW_P0_RESERVED      0xC0

// RX_PW_P1 register bits
#define NRF_RX_PW_P1_RESERVED      0xC0

// RX_PW_P2 register bits
#define NRF_RX_PW_P2_RESERVED      0xC0

// RX_PW_P3 register bits
#define NRF_RX_PW_P3_RESERVED      0xC0

// RX_PW_P4 register bits
#define NRF_RX_PW_P4_RESERVED      0xC0

// RX_PW_P5 register bits
#define NRF_RX_PW_P5_RESERVED      0xC0

// FIFO_STATUS register bits
#define NRF_FIFO_STATUS_RESERVED   0x8C
#define NRF_FIFO_STATUS_TX_REUSE   0x40
#define NRF_FIFO_STATUS_TX_FULL    0x20
#define NRF_FIFO_STATUS_TX_EMPTY   0x10
#define NRF_FIFO_STATUS_RX_FULL    0x02
#define NRF_FIFO_STATUS_RX_EMPTY   0x01

// IRQ
#define NRF_IRQ_RX_DR              NRF_STATUS_RX_DR
#define NRF_IRQ_TX_DS              NRF_STATUS_TX_DS
#define NRF_IRQ_MAX_RT             NRF_STATUS_MAX_RT

//-----------------------------------------------------------------

#endif /* NRF24L01_DEFS_H_ */
