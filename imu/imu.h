/*
 * imu.h
 *
 *  Created on: 13-10-2013
 *      Author: Korzo
 */

#ifndef IMU_H_
#define IMU_H_

//-----------------------------------------------------------------

void IMUConfig(void);

void IMUInit(float beta, float freq);

void IMUPollSensors(void);

void IMUUpdate(void);
void IMUEstimate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void IMUEstimateNoMag(float gx, float gy, float gz, float ax, float ay, float az);

void IMUGetEulerAngles(float *pitch, float *roll, float *yaw);

float invSqrt(float x);

//-----------------------------------------------------------------

#endif /* IMU_H_ */
