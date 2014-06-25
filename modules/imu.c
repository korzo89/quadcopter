/*
 * imu.c
 *
 *  Created on: 13-10-2013
 *      Author: Korzo
 */

#include "imu.h"

#include <stdint.h>
#include <math.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/pin_map.h>
#include <driverlib/gpio.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>

#include <drivers/imu_i2c.h>
#include <drivers/adxl345.h>
#include <drivers/l3g4200d.h>
#include <drivers/hmc5883.h>
#include <drivers/bmp085.h>
#include <modules/rcp.h>
#include <utils/delay.h>

//-----------------------------------------------------------------

#define IMU_TASK_STACK      300

#define PI                  3.141593f
#define RAD_TO_DEG(x)       ((x) * 57.2957795f)
#define DEG_TO_RAD(x)       ((x) * 0.01745329f)

#define TRIAD_REF_ACC_DEF   VEC3_NEW(-0.0156, 0.0391, 0.9375)
//#define TRIAD_REF_MAG_DEF   VEC3_NEW(191.2022, -48.0040, -367.2223)
#define TRIAD_REF_MAG_DEF   VEC3_NEW(131.6290, 11.9624, -343.7395)

//-----------------------------------------------------------------

typedef struct PACK_STRUCT
{
    vec3_t angles;
//    quat_t quat;
    vec3_t rates;
} cmd_angles_data_t;

//-----------------------------------------------------------------

static imu_sensor_data_t sensors;
static vec3_t angles;
static vec3_t rates;

// quaternion values
static float q0, q1, q2, q3;
static bool init_quat;

// filter parameters
static float filter_beta = 0.1f;
static float sample_freq = 100.0f;

volatile float pitch, roll, yaw;

// TRIAD reference vectors
static vec3_t triad_ref_acc = TRIAD_REF_ACC_DEF;
static vec3_t triad_ref_mag = TRIAD_REF_MAG_DEF;

// magnetometer calibration parameters
//float mag_calib_scale[] = {
//    0.727006, 0.0054922, 0.0269476,
//    0.0054922, 0.764489, 0.0283214,
//    0.0269476, 0.0283214, 0.993801
//};
//float mag_calib_offset[] = { -739.7, 334.691, 278.869 };
float mag_calib_scale[] = {
    0.70892, 0.00171642, 0.0316637,
    0.00171642, 0.770402, 0.0256191,
    0.0316637, 0.0256191, 0.993655
};
float mag_calib_offset[] = { 52.541, -147.339, -98.677 };

//-----------------------------------------------------------------

static void imu_poll_sensors(imu_sensor_data_t *data);

static result_t imu_quaternion_to_euler(vec3_t *out);

//-----------------------------------------------------------------

static void imu_rcp_callback_raw(rcp_message_t *msg)
{
	rcp_message_t resp;
	resp.packet.cmd = RCP_CMD_RAW_IMU;
	resp.packet.query = RCP_CMD_OK;

	memcpy(resp.packet.data, (uint8_t*)&sensors, sizeof(sensors));

	rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void imu_rcp_callback_angles(rcp_message_t *msg)
{
    rcp_message_t resp;
    resp.packet.cmd = RCP_CMD_ANGLES;
    resp.packet.query = RCP_CMD_OK;

    cmd_angles_data_t data;
    data.angles = angles;
//    data.quat = (quat_t){ q0, q1, q2, q3 };
    data.rates = rates;

    memcpy(resp.packet.data, &data, sizeof(data));

    rcp_send_message(&resp);
}

//-----------------------------------------------------------------

static void imu_task(void *params)
{
    portTickType last_wake = xTaskGetTickCount();

    while (1)
    {
        imu_update();

        vTaskDelayUntil(&last_wake, MSEC_TO_TICKS(10));
    }
}

//-----------------------------------------------------------------

result_t imu_init(float beta, float freq)
{
    imu_i2c_init();

    // init sensors
    adxl345_init();
    adxl345_set_offsets(0, 0, 0);
    l3g4200d_init();
    hmc5883_init();
    bmp085_init();

    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    init_quat = true;

    filter_beta = beta;
    sample_freq = freq;

    rcp_register_callback(RCP_CMD_RAW_IMU, imu_rcp_callback_raw, true);
    rcp_register_callback(RCP_CMD_ANGLES, imu_rcp_callback_angles, true);

    if (xTaskCreate(imu_task, (signed portCHAR*)"IMU",
            IMU_TASK_STACK, NULL, 2, NULL) != pdPASS)
        return RES_ERR_FATAL;

    return RES_OK;
}

//-----------------------------------------------------------------

static void imu_poll_sensors(imu_sensor_data_t *data)
{
    adxl345_get_accel(&data->acc.x, &data->acc.y, &data->acc.z);
    l3g4200d_read_gyro(&data->gyro.x, &data->gyro.y, &data->gyro.z);
    hmc5883_read_mag(&data->mag.x, &data->mag.y, &data->mag.z);
    data->temperature = bmp085_read_temp();
    data->pressure = bmp085_read_pressure();
}

//-----------------------------------------------------------------

result_t imu_get_sensors(imu_sensor_data_t *out)
{
    if (!out)
        return RES_ERR_BAD_PARAM;

    memcpy(out, &sensors, sizeof(imu_sensor_data_t));

    return RES_OK;
}

//-----------------------------------------------------------------

void imu_update(void)
{
    static imu_real_t real_sens;

    imu_poll_sensors(&sensors);
    imu_sensors_transform(&sensors, &real_sens);

    if (init_quat)
    {
        init_quat = false;

        quat_t est;
        imu_estimate_triad(real_sens.acc, real_sens.mag, &est, NULL);
        q0 = est.q0;
        q1 = est.q1;
        q2 = est.q2;
        q3 = est.q3;
    }

    vec3_t gyro = {
        .x = DEG_TO_RAD(real_sens.gyro.x),
        .y = DEG_TO_RAD(real_sens.gyro.y),
        .z = DEG_TO_RAD(real_sens.gyro.z)
    };

    imu_estimate_madgwick(&real_sens.acc, &real_sens.mag, &gyro);
    imu_quaternion_to_euler(&angles);

    rates = real_sens.gyro;
}

//-----------------------------------------------------------------

result_t imu_estimate_madgwick(vec3_t *acc, vec3_t *mag, vec3_t *gyro)
{
    if (!acc || !mag || !gyro)
        return RES_ERR_BAD_PARAM;

    float mx = mag->x, my = mag->y, mz = mag->z;
    // Use imu algorithm if magnetometer measurement invalid
    // (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
        return imu_estimate_madgwick_no_mag(acc, gyro);

    float ax = acc->x, ay = acc->y, az = acc->z;
    float gx = gyro->x, gy = gyro->y, gz = gyro->z;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3,
          _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= filter_beta * s0;
        qDot2 -= filter_beta * s1;
        qDot3 -= filter_beta * s2;
        qDot4 -= filter_beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sample_freq);
    q1 += qDot2 * (1.0f / sample_freq);
    q2 += qDot3 * (1.0f / sample_freq);
    q3 += qDot4 * (1.0f / sample_freq);

    // Normalise quaternion
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    return RES_OK;
}

//-----------------------------------------------------------------

result_t imu_estimate_madgwick_no_mag(vec3_t *acc, vec3_t *gyro)
{
    if (!acc || !gyro)
        return RES_ERR_BAD_PARAM;

    float ax = acc->x, ay = acc->y, az = acc->z;
    float gx = gyro->x, gy = gyro->y, gz = gyro->z;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= filter_beta * s0;
        qDot2 -= filter_beta * s1;
        qDot3 -= filter_beta * s2;
        qDot4 -= filter_beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sample_freq);
    q1 += qDot2 * (1.0f / sample_freq);
    q2 += qDot3 * (1.0f / sample_freq);
    q3 += qDot4 * (1.0f / sample_freq);

    // Normalise quaternion
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    return RES_OK;
}

//-----------------------------------------------------------------

static result_t imu_quaternion_to_euler(vec3_t *out)
{
    if (!out)
        return RES_ERR_BAD_PARAM;

//    float m11 = 2.0f * q0 * q0 - 1.0f + 2.0f * q1 * q1;
//    float m21 = 2.0f * (q1 * q2 - q0 * q3);
//    float m31 = 2.0f * (q1 * q3 + q0 * q2);
//    float m32 = 2.0f * (q2 * q3 - q0 * q1);
//    float m33 = 2.0f * q0 * q0 - 1.0f + 2.0f * q3 * q3;
//
//    // pitch
//    out->y = RAD_TO_DEG(-atanf(m31 * inv_sqrt(1.0f - m31 * m31)));
//    // roll
//    out->x = RAD_TO_DEG(atan2f(m32, m33));
//    // yaw
//    out->z = RAD_TO_DEG(atan2f(m21, m11));

    float a = 2.0f * (q0 * q1 + q2 * q3);
    float b = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    float c = 2.0f * (q0 * q2 - q1 * q3);
    float d = 2.0f * (q0 * q3 + q1 * q2);
    float e = 1.0f - 2.0f * (q2 * q2 + q3 * q3);

    // pitch
    out->y = RAD_TO_DEG(asinf(c));
    // roll
    out->x = RAD_TO_DEG(atan2f(a, b));
    // yaw
    out->z = RAD_TO_DEG(atan2f(d, e));

    return RES_OK;
}

//-----------------------------------------------------------------

result_t imu_sensors_transform(imu_sensor_data_t *sens, imu_real_t *real)
{
    if (!sens || !real)
        return RES_ERR_BAD_PARAM;

    real->acc.x = (float)sens->acc.x / 256.0;
    real->acc.y = (float)sens->acc.y / 256.0;
    real->acc.z = (float)sens->acc.z / 256.0;

    real->gyro.x = (float)sens->gyro.x * (70.0 / 1000.0);
    real->gyro.y = (float)sens->gyro.y * (70.0 / 1000.0);
    real->gyro.z = (float)sens->gyro.z * (70.0 / 1000.0);

    float mx = (float)sens->mag.x * 0.92 - mag_calib_offset[0];
    float my = (float)sens->mag.y * 0.92 - mag_calib_offset[1];
    float mz = (float)sens->mag.z * 0.92 - mag_calib_offset[2];

    float cx = mag_calib_scale[0] * mx + mag_calib_scale[1] * my + mag_calib_scale[2] * mz;
    float cy = mag_calib_scale[3] * mx + mag_calib_scale[4] * my + mag_calib_scale[5] * mz;
    float cz = mag_calib_scale[6] * mx + mag_calib_scale[7] * my + mag_calib_scale[8] * mz;
    mx = cx / 1000.0;
    my = cy / 1000.0;
    mz = cz / 1000.0;
    real->mag.x = mx;
    real->mag.y = my;
    real->mag.z = mz;

    return RES_OK;
}

//-----------------------------------------------------------------

result_t imu_get_angles(vec3_t *out)
{
    if (!out)
        return RES_ERR_BAD_PARAM;

    memcpy(out, &angles, sizeof(vec3_t));
    return RES_OK;
}

//-----------------------------------------------------------------

result_t imu_get_rates(vec3_t *out)
{
    if (!out)
        return RES_ERR_BAD_PARAM;

    memcpy(out, &rates, sizeof(vec3_t));
    return RES_OK;
}

//-----------------------------------------------------------------

result_t imu_estimate_triad(vec3_t acc, vec3_t mag, quat_t *quat, vec3_t *angles)
{
    float temp;
    vec3_t t1b = triad_ref_acc;
    VEC3_NORMALIZE_VAR(t1b, temp);
    vec3_t t1r = acc;
    VEC3_NORMALIZE_VAR(t1r, temp);
    vec3_t t2b = VEC3_CROSS(triad_ref_acc, triad_ref_mag);
    VEC3_NORMALIZE_VAR(t2b, temp);
    vec3_t t2r = VEC3_CROSS(acc, mag);
    VEC3_NORMALIZE_VAR(t2r, temp);
    vec3_t t3b = VEC3_CROSS(t1b, t2b);
    vec3_t t3r = VEC3_CROSS(t1r, t2r);

    float d11 = t1b.x*t1r.x + t2b.x*t2r.x + t3b.x*t3r.x;
    float d21 = t1b.y*t1r.x + t2b.y*t2r.x + t3b.y*t3r.x;
    float d31 = t1b.z*t1r.x + t2b.z*t2r.x + t3b.z*t3r.x;
    float d32 = t1b.z*t1r.y + t2b.z*t2r.y + t3b.z*t3r.y;
    float d33 = t1b.z*t1r.z + t2b.z*t2r.z + t3b.z*t3r.z;

    if (angles)
    {
        // pitch
        angles->y = RAD_TO_DEG(-atanf(d31 * inv_sqrt(1.0f - POW2(d31))));
        // roll
        angles->x = RAD_TO_DEG(atan2f(d32, d33));
        // yaw
        angles->z = RAD_TO_DEG(atan2f(d21, d11));
    }

    if (!quat)
        return RES_OK;

    float d12 = t1b.x*t1r.y + t2b.x*t2r.y + t3b.x*t3r.y;
    float d13 = t1b.x*t1r.z + t2b.x*t2r.z + t3b.x*t3r.z;
    float d22 = t1b.y*t1r.y + t2b.y*t2r.y + t3b.y*t3r.y;
    float d23 = t1b.y*t1r.z + t2b.y*t2r.z + t3b.y*t3r.z;

    float s;
    temp = d11 + d22 + d33; // D matrix trace
    if (temp > 0)
    {
        s = inv_sqrt(temp + 1.0f) / 2.0f;
        quat->q0 = 0.25f / s;
        quat->q1 = (d32 - d23) * s;
        quat->q2 = (d13 - d31) * s;
        quat->q3 = (d21 - d12) * s;
    }
    else if (d11 > d22 && d11 > d33)
    {
        s = inv_sqrt(1.0f + d11 - d22 - d33) / 2.0f;
        quat->q0 = (d32 - d23) * s;
        quat->q1 = 0.25f / s;
        quat->q2 = (d12 + d21) * s;
        quat->q3 = (d13 + d31) * s;
    }
    else if (d22 > d33)
    {
        s = inv_sqrt(1.0f + d22 - d11 - d33) / 2.0f;
        quat->q0 = (d13 - d31) * s;
        quat->q1 = (d12 + d21) * s;
        quat->q2 = 0.25f / s;
        quat->q3 = (d23 + d32) * s;
    }
    else
    {
        s = inv_sqrt(1.0f + d33 - d11 - d22) / 2.0f;
        quat->q0 = (d21 - d12) * s;
        quat->q1 = (d13 + d31) * s;
        quat->q2 = (d23 + d32) * s;
        quat->q3 = 0.25f / s;
    }

    return RES_OK;
}
