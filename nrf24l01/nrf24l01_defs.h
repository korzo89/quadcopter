/*
 * NRF24_defs.h
 *
 *  Created on: 30-10-2013
 *      Author: Korzo
 */

#ifndef NRF24_DEFS_H_
#define NRF24_DEFS_H_

//-----------------------------------------------------------------

//SPI command defines
#define NRF24_R_REGISTER             0x00
#define NRF24_W_REGISTER             0x20
#define NRF24_R_RX_PAYLOAD           0x61
#define NRF24_W_TX_PAYLOAD           0xA0
#define NRF24_FLUSH_TX               0xE1
#define NRF24_FLUSH_RX               0xE2
#define NRF24_REUSE_TX_PL            0xE3
#define NRF24_NOP                    0xFF

//SPI command data mask defines
#define NRF24_R_REGISTER_DATA        0x1F
#define NRF24_W_REGISTER_DATA        0x1F

////////////////////////////////////////////////////////////////////////////////////
// Register definitions
//
// Below are the defines for each register's address in the 24L01.
////////////////////////////////////////////////////////////////////////////////////
#define NRF24_CONFIG                 0x00
#define NRF24_EN_AA                  0x01
#define NRF24_EN_RXADDR              0x02
#define NRF24_SETUP_AW               0x03
#define NRF24_SETUP_RETR             0x04
#define NRF24_RF_CH                  0x05
#define NRF24_RF_SETUP               0x06
#define NRF24_STATUS                 0x07
#define NRF24_OBSERVE_TX             0x08
#define NRF24_CD                     0x09
#define NRF24_RX_ADDR_P0             0x0A
#define NRF24_RX_ADDR_P1             0x0B
#define NRF24_RX_ADDR_P2             0x0C
#define NRF24_RX_ADDR_P3             0x0D
#define NRF24_RX_ADDR_P4             0x0E
#define NRF24_RX_ADDR_P5             0x0F
#define NRF24_TX_ADDR                0x10
#define NRF24_RX_PW_P0               0x11
#define NRF24_RX_PW_P1               0x12
#define NRF24_RX_PW_P2               0x13
#define NRF24_RX_PW_P3               0x14
#define NRF24_RX_PW_P4               0x15
#define NRF24_RX_PW_P5               0x16
#define NRF24_FIFO_STATUS            0x17

////////////////////////////////////////////////////////////////////////////////////
// Default register values
//
// Below are the defines for each register's default value in the 24L01. Multi-byte
//   registers use notation B<X>, where "B" represents "byte" and <X> is the byte
//   number.
////////////////////////////////////////////////////////////////////////////////////
#define NRF24_CONFIG_DEFAULT_VAL             0x08
#define NRF24_EN_AA_DEFAULT_VAL              0x3F
#define NRF24_EN_RXADDR_DEFAULT_VAL          0x03
#define NRF24_SETUP_AW_DEFAULT_VAL           0x03
#define NRF24_SETUP_RETR_DEFAULT_VAL         0x03
#define NRF24_RF_CH_DEFAULT_VAL              0x02
#define NRF24_RF_SETUP_DEFAULT_VAL           0x0F
#define NRF24_STATUS_DEFAULT_VAL             0x0E
#define NRF24_OBSERVE_TX_DEFAULT_VAL         0x00
#define NRF24_CD_DEFAULT_VAL                 0x00
#define NRF24_RX_ADDR_P0_B0_DEFAULT_VAL      0xE7
#define NRF24_RX_ADDR_P0_B1_DEFAULT_VAL      0xE7
#define NRF24_RX_ADDR_P0_B2_DEFAULT_VAL      0xE7
#define NRF24_RX_ADDR_P0_B3_DEFAULT_VAL      0xE7
#define NRF24_RX_ADDR_P0_B4_DEFAULT_VAL      0xE7
#define NRF24_RX_ADDR_P1_B0_DEFAULT_VAL      0xC2
#define NRF24_RX_ADDR_P1_B1_DEFAULT_VAL      0xC2
#define NRF24_RX_ADDR_P1_B2_DEFAULT_VAL      0xC2
#define NRF24_RX_ADDR_P1_B3_DEFAULT_VAL      0xC2
#define NRF24_RX_ADDR_P1_B4_DEFAULT_VAL      0xC2
#define NRF24_RX_ADDR_P2_DEFAULT_VAL         0xC3
#define NRF24_RX_ADDR_P3_DEFAULT_VAL         0xC4
#define NRF24_RX_ADDR_P4_DEFAULT_VAL         0xC5
#define NRF24_RX_ADDR_P5_DEFAULT_VAL         0xC6
#define NRF24_TX_ADDR_B0_DEFAULT_VAL         0xE7
#define NRF24_TX_ADDR_B1_DEFAULT_VAL         0xE7
#define NRF24_TX_ADDR_B2_DEFAULT_VAL         0xE7
#define NRF24_TX_ADDR_B3_DEFAULT_VAL         0xE7
#define NRF24_TX_ADDR_B4_DEFAULT_VAL         0xE7
#define NRF24_RX_PW_P0_DEFAULT_VAL           0x00
#define NRF24_RX_PW_P1_DEFAULT_VAL           0x00
#define NRF24_RX_PW_P2_DEFAULT_VAL           0x00
#define NRF24_RX_PW_P3_DEFAULT_VAL           0x00
#define NRF24_RX_PW_P4_DEFAULT_VAL           0x00
#define NRF24_RX_PW_P5_DEFAULT_VAL           0x00
#define NRF24_FIFO_STATUS_DEFAULT_VAL        0x11

////////////////////////////////////////////////////////////////////////////////////
// Register bitwise definitions
//
// Below are the defines for each register's bitwise fields in the 24L01.
////////////////////////////////////////////////////////////////////////////////////
//CONFIG register bitwise definitions
#define NRF24_CONFIG_RESERVED        0x80
#define NRF24_CONFIG_MASK_RX_DR      0x40
#define NRF24_CONFIG_MASK_TX_DS      0x20
#define NRF24_CONFIG_MASK_MAX_RT     0x10
#define NRF24_CONFIG_EN_CRC          0x08
#define NRF24_CONFIG_CRCO            0x04
#define NRF24_CONFIG_PWR_UP          0x02
#define NRF24_CONFIG_PRIM_RX         0x01

//EN_AA register bitwise definitions
#define NRF24_EN_AA_RESERVED         0xC0
#define NRF24_EN_AA_ENAA_ALL         0x3F
#define NRF24_EN_AA_ENAA_P5          0x20
#define NRF24_EN_AA_ENAA_P4          0x10
#define NRF24_EN_AA_ENAA_P3          0x08
#define NRF24_EN_AA_ENAA_P2          0x04
#define NRF24_EN_AA_ENAA_P1          0x02
#define NRF24_EN_AA_ENAA_P0          0x01
#define NRF24_EN_AA_ENAA_NONE        0x00

//EN_RXADDR register bitwise definitions
#define NRF24_EN_RXADDR_RESERVED     0xC0
#define NRF24_EN_RXADDR_ERX_ALL      0x3F
#define NRF24_EN_RXADDR_ERX_P5       0x20
#define NRF24_EN_RXADDR_ERX_P4       0x10
#define NRF24_EN_RXADDR_ERX_P3       0x08
#define NRF24_EN_RXADDR_ERX_P2       0x04
#define NRF24_EN_RXADDR_ERX_P1       0x02
#define NRF24_EN_RXADDR_ERX_P0       0x01
#define NRF24_EN_RXADDR_ERX_NONE     0x00

//SETUP_AW register bitwise definitions
#define NRF24_SETUP_AW_RESERVED      0xFC
#define NRF24_SETUP_AW               0x03
#define NRF24_SETUP_AW_5BYTES        0x03
#define NRF24_SETUP_AW_4BYTES        0x02
#define NRF24_SETUP_AW_3BYTES        0x01
#define NRF24_SETUP_AW_ILLEGAL       0x00

//SETUP_RETR register bitwise definitions
#define NRF24_SETUP_RETR_ARD         0xF0
#define NRF24_SETUP_RETR_ARD_4000    0xF0
#define NRF24_SETUP_RETR_ARD_3750    0xE0
#define NRF24_SETUP_RETR_ARD_3500    0xD0
#define NRF24_SETUP_RETR_ARD_3250    0xC0
#define NRF24_SETUP_RETR_ARD_3000    0xB0
#define NRF24_SETUP_RETR_ARD_2750    0xA0
#define NRF24_SETUP_RETR_ARD_2500    0x90
#define NRF24_SETUP_RETR_ARD_2250    0x80
#define NRF24_SETUP_RETR_ARD_2000    0x70
#define NRF24_SETUP_RETR_ARD_1750    0x60
#define NRF24_SETUP_RETR_ARD_1500    0x50
#define NRF24_SETUP_RETR_ARD_1250    0x40
#define NRF24_SETUP_RETR_ARD_1000    0x30
#define NRF24_SETUP_RETR_ARD_750     0x20
#define NRF24_SETUP_RETR_ARD_500     0x10
#define NRF24_SETUP_RETR_ARD_250     0x00
#define NRF24_SETUP_RETR_ARC         0x0F
#define NRF24_SETUP_RETR_ARC_15      0x0F
#define NRF24_SETUP_RETR_ARC_14      0x0E
#define NRF24_SETUP_RETR_ARC_13      0x0D
#define NRF24_SETUP_RETR_ARC_12      0x0C
#define NRF24_SETUP_RETR_ARC_11      0x0B
#define NRF24_SETUP_RETR_ARC_10      0x0A
#define NRF24_SETUP_RETR_ARC_9       0x09
#define NRF24_SETUP_RETR_ARC_8       0x08
#define NRF24_SETUP_RETR_ARC_7       0x07
#define NRF24_SETUP_RETR_ARC_6       0x06
#define NRF24_SETUP_RETR_ARC_5       0x05
#define NRF24_SETUP_RETR_ARC_4       0x04
#define NRF24_SETUP_RETR_ARC_3       0x03
#define NRF24_SETUP_RETR_ARC_2       0x02
#define NRF24_SETUP_RETR_ARC_1       0x01
#define NRF24_SETUP_RETR_ARC_0       0x00

//RF_CH register bitwise definitions
#define NRF24_RF_CH_RESERVED 0x80

//RF_SETUP register bitwise definitions
#define NRF24_RF_SETUP_RESERVED      0xE0
#define NRF24_RF_SETUP_PLL_LOCK      0x10
#define NRF24_RF_SETUP_RF_DR         0x08
#define NRF24_RF_SETUP_RF_PWR        0x06
#define NRF24_RF_SETUP_RF_PWR_0      0x06
#define NRF24_RF_SETUP_RF_PWR_6      0x04
#define NRF24_RF_SETUP_RF_PWR_12     0x02
#define NRF24_RF_SETUP_RF_PWR_18     0x00
#define NRF24_RF_SETUP_LNA_HCURR     0x01

//STATUS register bitwise definitions
#define NRF24_STATUS_RESERVED                   0x80
#define NRF24_STATUS_RX_DR                      0x40
#define NRF24_STATUS_TX_DS                      0x20
#define NRF24_STATUS_MAX_RT                     0x10
#define NRF24_STATUS_RX_P_NO                    0x0E
#define NRF24_STATUS_RX_P_NO_RX_FIFO_NOT_EMPTY  0x0E
#define NRF24_STATUS_RX_P_NO_UNUSED             0x0C
#define NRF24_STATUS_RX_P_NO_5                  0x0A
#define NRF24_STATUS_RX_P_NO_4                  0x08
#define NRF24_STATUS_RX_P_NO_3                  0x06
#define NRF24_STATUS_RX_P_NO_2                  0x04
#define NRF24_STATUS_RX_P_NO_1                  0x02
#define NRF24_STATUS_RX_P_NO_0                  0x00
#define NRF24_STATUS_TX_FULL                    0x01

//OBSERVE_TX register bitwise definitions
#define NRF24_OBSERVE_TX_PLOS_CNT    0xF0
#define NRF24_OBSERVE_TX_ARC_CNT     0x0F

//CD register bitwise definitions
#define NRF24_CD_RESERVED            0xFE
#define NRF24_CD_CD                  0x01

//RX_PW_P0 register bitwise definitions
#define NRF24_RX_PW_P0_RESERVED      0xC0

//RX_PW_P0 register bitwise definitions
#define NRF24_RX_PW_P0_RESERVED      0xC0

//RX_PW_P1 register bitwise definitions
#define NRF24_RX_PW_P1_RESERVED      0xC0

//RX_PW_P2 register bitwise definitions
#define NRF24_RX_PW_P2_RESERVED      0xC0

//RX_PW_P3 register bitwise definitions
#define NRF24_RX_PW_P3_RESERVED      0xC0

//RX_PW_P4 register bitwise definitions
#define NRF24_RX_PW_P4_RESERVED      0xC0

//RX_PW_P5 register bitwise definitions
#define NRF24_RX_PW_P5_RESERVED      0xC0

//FIFO_STATUS register bitwise definitions
#define NRF24_FIFO_STATUS_RESERVED   0x8C
#define NRF24_FIFO_STATUS_TX_REUSE   0x40
#define NRF24_FIFO_STATUS_TX_FULL    0x20
#define NRF24_FIFO_STATUS_TX_EMPTY   0x10
#define NRF24_FIFO_STATUS_RX_FULL    0x02
#define NRF24_FIFO_STATUS_RX_EMPTY   0x01

//-----------------------------------------------------------------

#endif /* NRF24_DEFS_H_ */
