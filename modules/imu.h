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

typedef struct PACK_STRUCT
{
    int16_t accX, accY, accZ;       // raw accelerometer values
    int16_t gyroX, gyroY, gyroZ;    // raw gyroscope values
    int16_t magX, magY, magZ;       // raw magnetometer values
    int32_t pressure;               // raw pressure
    int16_t temperature;            // raw temperature
} IMUSensorData;

//-----------------------------------------------------------------

void imuInit(float beta, float freq);

void imuPollSensors(IMUSensorData *data);

void imuUpdate(void);
void imuEstimate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void imuEstimateNoMag(float gx, float gy, float gz, float ax, float ay, float az);

void imuGetEulerAngles(float *pitch, float *roll, float *yaw);

float invSqrt(float x);

//-----------------------------------------------------------------

#endif /* IMU_H_ */
