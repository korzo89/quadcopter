/*
 * hmc5883.h
 *
 *  Created on: 24-09-2013
 *      Author: Korzo
 */

#ifndef HMC5883_H_
#define HMC5883_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

#define HMC5883_CONIFG_A        0x00
#define HMC5883_CONIFG_B        0x01
#define HMC5883_MODE            0x02
#define HMC5883_OUT_X_MSB       0x03
#define HMC5883_OUT_X_LSB       0x04
#define HMC5883_OUT_Z_MSB       0x05
#define HMC5883_OUT_Z_LSB       0x06
#define HMC5883_OUT_Y_MSB       0x07
#define HMC5883_OUT_Y_LSB       0x08
#define HMC5883_STATUS          0x09
#define HMC5883_ID_A            0x0A
#define HMC5883_ID_B            0x0B
#define HMC5883_ID_C            0x0C

//-----------------------------------------------------------------

void hmc5883_init();

void hmc5883_read_mag(int16_t *x, int16_t *y, int16_t *z);

//-----------------------------------------------------------------

#endif /* HMC5883_H_ */
