/*
 * ext_i2c.c
 *
 *  Created on: 20 maj 2014
 *      Author: Korzo
 */

#include "ext_i2c.h"

#include <stellaris_config.h>

//-----------------------------------------------------------------

static i2c_t i2c_if = {
    .conf = {
         .fast = true,
         .base = I2C2_MASTER_BASE,
         .sysctl = SYSCTL_PERIPH_I2C2,
         .scl = {
             .port = GPIO_PORTE_BASE,
             .pin = GPIO_PIN_4,
             .sysctl = SYSCTL_PERIPH_GPIOE,
             .conf = GPIO_PE4_I2C2SCL
         },
         .sda = {
              .port = GPIO_PORTE_BASE,
              .pin = GPIO_PIN_5,
              .sysctl = SYSCTL_PERIPH_GPIOE,
              .conf = GPIO_PE5_I2C2SDA
          }
    }
};

//-----------------------------------------------------------------

void ext_i2c_init(void)
{
    i2c_init(&i2c_if);
}

//-----------------------------------------------------------------

i2c_t* ext_i2c_get_if(void)
{
    return &i2c_if;
}
