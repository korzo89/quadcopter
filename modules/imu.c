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

//-----------------------------------------------------------------

#define PI              3.141593f
#define RAD_TO_DEG(x)   ((x) * 57.2957795f)
#define DEG_TO_RAD(x)   ((x) * 0.01745329f)

//-----------------------------------------------------------------

static imu_sensor_data_t curr_data;

// quaternion values
static float q0 = 1.0f, q1, q2, q3;

// filter parameters
static float filter_beta = 0.1f;
static float sample_freq = 100.0f;

// magnetometer calibration parameters
static const float mag_cal[] = { 0.727006, 0.0054922, 0.0269476,
                                 0.0054922, 0.764489, 0.0283214,
                                 0.0269476, 0.0283214, 0.993801 };
static const float mag_off[] = { -739.7, 334.691, 278.869 };

volatile float pitch, roll, yaw;

//-----------------------------------------------------------------

static void imu_rcp_callback(rcp_message_t *msg)
{
	rcp_message_t resp;
	resp.packet.cmd = RCP_CMD_RAW_IMU;
	resp.packet.query = RCP_CMD_OK;

	imu_poll_sensors((imu_sensor_data_t*)resp.packet.data);

	rcp_send_message(&resp);
}

//-----------------------------------------------------------------

void imu_init(float beta, float freq)
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

    filter_beta = beta;
    sample_freq = freq;

    rcp_register_callback(RCP_CMD_RAW_IMU, imu_rcp_callback, true);
}

//-----------------------------------------------------------------

void imu_poll_sensors(imu_sensor_data_t *data)
{
    adxl345_get_accel(&data->acc.x, &data->acc.y, &data->acc.z);
    l3g4200d_read_gyro(&data->gyro.x, &data->gyro.y, &data->gyro.z);
    hmc5883_read_mag(&data->mag.x, &data->mag.y, &data->mag.z);
//    data->temperature = bmp085ReadTemperature();
//    data->pressure = bmp085ReadPressure();
}

//-----------------------------------------------------------------

static float inv_sqrt(float x)
{
    unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
    float tmp = *(float*)&i;
    return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}

//-----------------------------------------------------------------

void imu_update(void)
{
    float ax, ay, az, gx, gy, gz, mx, my, mz, cx, cy, cz;

    ax = (float) curr_data.acc.x / 256.0;
    ay = (float) curr_data.acc.y / 256.0;
    az = (float) curr_data.acc.z / 256.0;

    gx = DEG_TO_RAD((float) curr_data.gyro.x * 70.0 / 1000.0);
    gy = DEG_TO_RAD((float) curr_data.gyro.y * 70.0 / 1000.0);
    gz = DEG_TO_RAD((float) curr_data.gyro.z * 70.0 / 1000.0);

    mx = (float) curr_data.mag.x * 0.92 - mag_off[0];
    my = (float) curr_data.mag.y * 0.92 - mag_off[1];
    mz = (float) curr_data.mag.z * 0.92 - mag_off[2];

    cx = mag_cal[0]*mx + mag_cal[1]*my + mag_cal[2]*mz;
    cy = mag_cal[3]*mx + mag_cal[4]*my + mag_cal[5]*mz;
    cz = mag_cal[6]*mx + mag_cal[7]*my + mag_cal[8]*mz;
    mx = cx / 1000.0;
    my = cy / 1000.0;
    mz = cz / 1000.0;

    imu_estimate(gx, gy, gz, ax, ay, az, mx, my, mz);
}

//-----------------------------------------------------------------

void imu_estimate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use imu algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        imu_estimate_no_mag(gx, gy, gz, ax, ay, az);
        return;
    }

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
}

//-----------------------------------------------------------------

void imu_estimate_no_mag(float gx, float gy, float gz, float ax, float ay, float az)
{
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
}

//-----------------------------------------------------------------

void imu_get_euler_angles(float *pitch, float *roll, float *yaw)
{
    float m11 = 2.0f * q0 * q0 - 1.0f + 2.0f * q1 * q1;
    float m21 = 2.0f * (q1 * q2 - q0 * q3);
    float m31 = 2.0f * (q1 * q3 + q0 * q2);
    float m32 = 2.0f * (q2 * q3 - q0 * q1);
    float m33 = 2.0f * q0 * q0 - 1.0f + 2.0f * q3 * q3;

    *roll  = RAD_TO_DEG(atan2f(m32, m33));
    *pitch = RAD_TO_DEG(-atanf(m31 * inv_sqrt(1.0f - m31 * m31)));
    *yaw   = RAD_TO_DEG(atan2f(m21, m11));
}
