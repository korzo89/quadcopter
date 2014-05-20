/*
 * ext_i2c.h
 *
 *  Created on: 20 maj 2014
 *      Author: Korzo
 */

#ifndef EXT_I2C_H_
#define EXT_I2C_H_


//-----------------------------------------------------------------

#include <drivers/i2c.h>

//-----------------------------------------------------------------

void ext_i2c_init(void);

i2c_t* ext_i2c_get_if(void);

//-----------------------------------------------------------------

#endif /* EXT_I2C_H_ */
