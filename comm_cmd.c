#include "comm_cmd.h"
#include "comm_buffer.h"
#include <hal/imu.h>
#include <modules/pid.h>
#include "control.h"
#include <drivers/motors.h>

#include <stdint.h>

//-----------------------------------------------------------------

extern CommBuffer buffer;

extern int16_t accX, accY, accZ;       // raw accelerometer values
extern int16_t gyroX, gyroY, gyroZ;    // raw gyroscope values
extern int16_t magX, magY, magZ;       // raw magnetometer values
extern int16_t temperature;            // raw temperature
extern int32_t pressure;               // raw pressure

//-----------------------------------------------------------------

typedef void (*CommHandler)(void);

static const CommHandler cmdHandlers[] = {
    0,          // MSG_CMD_OK
    cmdControl, // MSG_CMD_CONTROL
    cmdArm,     // MSG_CMD_ARM
    cmdDisarm,  // MSG_CMD_DISARM
    0,          // MSG_CMD_RAW_IMU
    0,          // MSG_CMD_ANGLES
    0,          // MSG_CMD_SET_PID_PITCH
    0,          // MSG_CMD_SET_PID_ROLL
    0,          // MSG_CMD_SET_PID_YAW
};

static const CommHandler respHandlers[] = {
    respOK,     // MSG_CMD_OK
    0,          // MSG_CMD_CONTROL
    0,          // MSG_CMD_ARM
    0,          // MSG_CMD_DISARM
    respRawIMU, // MSG_CMD_RAW_IMU
    respAngles, // MSG_CMD_ANGLES
    0,          // MSG_CMD_SET_PID_PITCH
    0,          // MSG_CMD_SET_PID_ROLL
    0           // MSG_CMD_SET_PID_YAW
};

//-----------------------------------------------------------------

void commHandleMessage()
{
    unsigned int code;
    CommHandler handler;

    // handle incoming command
    code = (int) buffer.command;
    if (code < (int) MSG_CMD_COUNT)
    {
        handler = cmdHandlers[code];
        if (handler)
            handler();
    }

    // handle response request
    code = (int) buffer.response;
    if (code < (int) MSG_CMD_COUNT)
    {
        handler = respHandlers[code];
        if (handler)
            handler();
    }
}

//-----------------------------------------------------------------

void cmdControl(void)
{
    uint16_t m1, m2, m3, m4;

    if (!motorsArmed())
        return;

    commBufferResetRead(&buffer);
    COMM_BUFFER_READ_T(&buffer, &m1, uint16_t);
    COMM_BUFFER_READ_T(&buffer, &m2, uint16_t);
    COMM_BUFFER_READ_T(&buffer, &m3, uint16_t);
    COMM_BUFFER_READ_T(&buffer, &m4, uint16_t);

    motorsSetThrottle((float)m1, (float)m2, (float)m3, (float)m4);
}

//-----------------------------------------------------------------

void cmdArm(void)
{
    motorsArm();
}

//-----------------------------------------------------------------

void cmdDisarm(void)
{
    motorsDisarm();
}

//-----------------------------------------------------------------

void cmdSetPIDPitch(void)
{
    bool en;
    float kp, ki, kd, mo, mi;

    cmdExtractPID(&en, &kp, &ki, &kd, &mo, &mi);
    controlSetPitchPID(en, kp, ki, kd, mo, 0.0f, mi);
}

//-----------------------------------------------------------------

void cmdSetPIDRoll(void)
{
    bool en;
    float kp, ki, kd, mo, mi;

    cmdExtractPID(&en, &kp, &ki, &kd, &mo, &mi);
    controlSetRollPID(en, kp, ki, kd, mo, 0.0f, mi);
}

//-----------------------------------------------------------------

void cmdSetPIDYaw(void)
{
    bool en;
    float kp, ki, kd, mo, mi;
    cmdExtractPID(&en, &kp, &ki, &kd, &mo, &mi);
    controlSetYawPID(en, kp, ki, kd, mo, 0.0f, mi);
}

//-----------------------------------------------------------------

void cmdExtractPID(bool *en, float *kp, float *ki, float *kd, float *mo, float *mi)
{
    unsigned char temp;

    commBufferResetRead(&buffer);
    COMM_BUFFER_READ_T(&buffer, &temp, unsigned char);
    *en = (bool) temp;
    COMM_BUFFER_READ_T(&buffer, kp, float);
    COMM_BUFFER_READ_T(&buffer, ki, float);
    COMM_BUFFER_READ_T(&buffer, kd, float);
    COMM_BUFFER_READ_T(&buffer, mo, float);
    COMM_BUFFER_READ_T(&buffer, mi, float);
}

//-----------------------------------------------------------------

void respOK(void)
{
    commBufferCreateHeader(&buffer, MSG_CMD_OK, MSG_CMD_OK);
}

//-----------------------------------------------------------------

void respRawIMU(void)
{
    commBufferResetWrite(&buffer);
    COMM_BUFFER_WRITE_T(&buffer, &accX, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &accY, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &accZ, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &gyroX, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &gyroY, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &gyroZ, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &magX, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &magY, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &magZ, uint16_t);
    COMM_BUFFER_WRITE_T(&buffer, &pressure, uint32_t);
    COMM_BUFFER_WRITE_T(&buffer, &temperature, uint16_t);
    commBufferCreateHeader(&buffer, MSG_CMD_RAW_IMU, MSG_CMD_OK);
}

//-----------------------------------------------------------------

void respAngles(void)
{
    float pitch, roll, yaw;

    imuGetEulerAngles(&pitch, &roll, &yaw);

    commBufferResetWrite(&buffer);
    COMM_BUFFER_WRITE_T(&buffer, &pitch, float);
    COMM_BUFFER_WRITE_T(&buffer, &roll, float);
    COMM_BUFFER_WRITE_T(&buffer, &yaw, float);
    commBufferCreateHeader(&buffer, MSG_CMD_ANGLES, MSG_CMD_OK);
}
