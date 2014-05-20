/*
 * imu.h
 *
 *  Created on: 13-10-2013
 *      Author: Korzo
 */

#ifndef IMU_H_
#define IMU_H_

//-----------------------------------------------------------------

#include <defs.h>

//-----------------------------------------------------------------

struct PACK_STRUCT imu_sensor_vec3
{
    int16_t x, y, z;
};

typedef struct PACK_STRUCT
{
    struct imu_sensor_vec3 acc;     // raw accelerometer values
    struct imu_sensor_vec3 gyro;    // raw gyroscope values
    struct imu_sensor_vec3 mag;     // raw magnetometer values
    int32_t pressure;               // raw pressure
    int16_t temperature;            // raw temperature
} imu_sensor_data_t;

//-----------------------------------------------------------------

void imu_init(float beta, float freq);

void imu_poll_sensors(imu_sensor_data_t *data);

void imu_update(void);
void imu_estimate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void imu_estimate_no_mag(float gx, float gy, float gz, float ax, float ay, float az);

void imu_get_euler_angles(float *pitch, float *roll, float *yaw);

//-----------------------------------------------------------------

#endif /* IMU_H_ */
