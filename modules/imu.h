/*
 * imu.h
 *
 *  Created on: 13-10-2013
 *      Author: Korzo
 */

#ifndef IMU_H_
#define IMU_H_

//-----------------------------------------------------------------

void imuConfig(void);
void imuInit(float beta, float freq);

void imuPollSensors(void);

void imuUpdate(void);
void imuEstimate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void imuEstimateNoMag(float gx, float gy, float gz, float ax, float ay, float az);

void imuGetEulerAngles(float *pitch, float *roll, float *yaw);

float invSqrt(float x);

//-----------------------------------------------------------------

#endif /* IMU_H_ */
