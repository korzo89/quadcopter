/*
 * adxl345.h
 *
 *  Created on: 23-09-2013
 *      Author: Korzo
 */

#ifndef ADXL345_H_
#define ADXL345_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

// ADXL345 registers
#define ADXL345_DEVID           0x00
#define ADXL345_THRESH_TAP      0x1D
#define ADXL345_OFSX            0x1E
#define ADXL345_OFSY            0x1F
#define ADXL345_OFSZ            0x20
#define ADXL345_DUR             0x21
#define ADXL345_LATENT          0x22
#define ADXL345_WINDOW          0x23
#define ADXL345_THRESH_ACT      0x24
#define ADXL345_THRESH_INACT    0x25
#define ADXL345_TIME_INACT      0x26
#define ADXL345_ACT_INACT_CTL   0x27
#define ADXL345_THRESH_FF       0x28
#define ADXL345_TIME_FF         0x29
#define ADXL345_TAP_AXES        0x2A
#define ADXL345_ACT_TAP_STATUS  0x2B
#define ADXL345_BW_RATE         0x2C
#define ADXL345_POWER_CTL       0x2D
#define ADXL345_INT_ENABLE      0x2E
#define ADXL345_INT_MAP         0x2F
#define ADXL345_INT_SOURCE      0x30
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_DATAX0          0x32
#define ADXL345_DATAX1          0x33
#define ADXL345_DATAY0          0x34
#define ADXL345_DATAY1          0x35
#define ADXL345_DATAZ0          0x36
#define ADXL345_DATAZ1          0x37
#define ADXL345_FIFO_CTL        0x38
#define ADXL345_FIFO_STATUS     0x39

// ADXL345 bits
#define ADXL345_SLEEP           (1 << 2)
#define ADXL345_MEASURE         (1 << 3)
#define ADXL345_AUTO_SLEEP      (1 << 4)
#define ADXL345_LINK            (1 << 5)

#define ADXL345_OVERRUN         (1 << 0)
#define ADXL345_WATERMARK       (1 << 1)
#define ADXL345_FREE_FALL       (1 << 2)
#define ADXL345_INACTIVITY      (1 << 3)
#define ADXL345_ACTIVITY        (1 << 4)
#define ADXL345_DOUBLE_TAP      (1 << 5)
#define ADXL345_SINGLE_TAP      (1 << 6)
#define ADXL345_DATA_READY      (1 << 7)

#define ADXL345_JUSTIFY         (1 << 2)
#define ADXL345_FULL_RES        (1 << 3)
#define ADXL345_INT_INVERT      (1 << 5)

#define ADXL345_RANGE_MASK      0x03

//-----------------------------------------------------------------

typedef enum
{
    ADXL345_RANGE_2G    = 0x00,
    ADXL345_RANGE_4G    = 0x01,
    ADXL345_RANGE_8G    = 0x02,
    ADXL345_RANGE_16G   = 0x03
} adxl345_range_t;

//-----------------------------------------------------------------

void adxl345_init(void);

void adxl345_get_accel(int16_t *x, int16_t *y, int16_t *z);

adxl345_range_t adxl345_get_range();
void adxl345_set_range(adxl345_range_t range);

void adxl345_get_offsets(int8_t *x, int8_t *y, int8_t *z);
void adxl345_set_offsets(int8_t x, int8_t y, int8_t z);

uint8_t adxl345_get_int_source();

uint8_t adxl345_get_int_mapping();
void adxl345_set_int_mapping(uint8_t map);

//-----------------------------------------------------------------

#endif /* ADXL345_H_ */
