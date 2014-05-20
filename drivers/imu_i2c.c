#include "imu_i2c.h"

#include <stellaris_config.h>

//-----------------------------------------------------------------

static i2c_t i2c_if = {
    .conf = {
         .fast = true,
         .base = I2C0_MASTER_BASE,
         .sysctl = SYSCTL_PERIPH_I2C0,
         .scl = {
             .port = GPIO_PORTB_BASE,
             .pin = GPIO_PIN_2,
             .sysctl = SYSCTL_PERIPH_GPIOB,
             .conf = GPIO_PB2_I2C0SCL
         },
         .sda = {
              .port = GPIO_PORTB_BASE,
              .pin = GPIO_PIN_3,
              .sysctl = SYSCTL_PERIPH_GPIOB,
              .conf = GPIO_PB3_I2C0SDA
          }
    }
};

//-----------------------------------------------------------------

void imu_i2c_init(void)
{
    i2c_init(&i2c_if);
}

//-----------------------------------------------------------------

i2c_t* imu_i2c_get_if(void)
{
    return &i2c_if;
}
