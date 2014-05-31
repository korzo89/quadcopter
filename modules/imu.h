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
#include <utils/vec3.h>

//-----------------------------------------------------------------

typedef struct PACK_STRUCT
{
    int16_t x, y, z;
} sensor_vec3_t;

typedef struct PACK_STRUCT
{
    sensor_vec3_t acc;     // raw accelerometer values
    sensor_vec3_t gyro;    // raw gyroscope values
    sensor_vec3_t mag;     // raw magnetometer values
    int32_t pressure;      // raw pressure
    int16_t temperature;   // raw temperature
} imu_sensor_data_t;

typedef struct
{
    vec3_t acc;
    vec3_t mag;
    vec3_t gyro;
} imu_real_t;

//-----------------------------------------------------------------

void imu_init(float beta, float freq);

void imu_poll_sensors(imu_sensor_data_t *data);

void imu_update(void);
void imu_estimate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void imu_estimate_no_mag(float gx, float gy, float gz, float ax, float ay, float az);

void imu_get_euler_angles(float *pitch, float *roll, float *yaw);

result_t imu_sensors_transform(imu_sensor_data_t *sens, imu_real_t *real);
result_t imu_estimate_triad(vec3_t acc, vec3_t mag, vec3_t *out);

//-----------------------------------------------------------------

#endif /* IMU_H_ */
