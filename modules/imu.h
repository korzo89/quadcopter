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

typedef struct
{
    float q0, q1, q2, q3;
} quat_t;

//-----------------------------------------------------------------

result_t imu_init(float beta, float freq);

void imu_update(void);

result_t imu_get_sensors(imu_sensor_data_t *out);
result_t imu_sensors_transform(imu_sensor_data_t *sens, imu_real_t *real);

result_t imu_get_angles(vec3_t *out);
result_t imu_get_rates(vec3_t *out);

result_t imu_estimate_triad(vec3_t acc, vec3_t mag, quat_t *quat, vec3_t *angles);
result_t imu_estimate_madgwick(vec3_t *acc, vec3_t *mag, vec3_t *gyro);
result_t imu_estimate_madgwick_no_mag(vec3_t *acc, vec3_t *gyro);

//-----------------------------------------------------------------

#endif /* IMU_H_ */
