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

struct sensor_vec3
{
    int16_t x, y, z;
};

struct imu_sensor_data
{
    struct sensor_vec3 acc;
    struct sensor_vec3 gyro;
    struct sensor_vec3 mag;
    int32_t pressure;
    int16_t temperature;
};

struct imu_real
{
    struct vec3 acc;
    struct vec3 mag;
    struct vec3 gyro;
};

struct quat
{
    float q0, q1, q2, q3;
};

//-----------------------------------------------------------------

result_t imu_init(void);

void imu_update(void);

result_t imu_get_sensors(struct imu_sensor_data *out);
result_t imu_sensors_transform(struct imu_sensor_data *sens, struct imu_real *real);

result_t imu_get_angles(struct vec3 *out);
result_t imu_get_rates(struct vec3 *out);

result_t imu_estimate_triad(struct vec3 acc, struct vec3 mag, struct quat *quat, struct vec3 *angles);
result_t imu_estimate_madgwick(struct vec3 *acc, struct vec3 *mag, struct vec3 *gyro);
result_t imu_estimate_madgwick_no_mag(struct vec3 *acc, struct vec3 *gyro);

//-----------------------------------------------------------------

#endif /* IMU_H_ */
