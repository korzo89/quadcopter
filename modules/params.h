/*
 * params.h
 *
 *  Created on: 14 cze 2014
 *      Author: Korzo
 */

#ifndef PARAMS_H_
#define PARAMS_H_

//-----------------------------------------------------------------

#include <defs.h>
#include <utils/pid.h>
#include <utils/vec3.h>
#include <modules/control.h>

//-----------------------------------------------------------------

enum param_type
{
    PARAM_TYPE_UINT8 = 0,
    PARAM_TYPE_UINT16,
    PARAM_TYPE_UINT32,
    PARAM_TYPE_INT8,
    PARAM_TYPE_INT16,
    PARAM_TYPE_INT32,
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_ENUM
};

struct param_info
{
    const char      *group;     // max group len: 14 chars + zero = 15
    const char      *name;      // max name len: 10 chars + zero = 11
    const char      *meta;
    enum param_type type;
    uint8_t         size;
    uint8_t         count;
    void            *ptr;
};

//-----------------------------------------------------------------

void params_init(void);
void params_load_defaults(void);

result_t params_eeprom_save(void);
result_t params_eeprom_load(void);

result_t params_get_pid_pitch(struct pid_params *out);
result_t params_get_pid_roll(struct pid_params *out);
result_t params_get_pid_yaw(struct pid_params *out);
result_t params_get_pid_pitch_rate(struct pid_params *out);
result_t params_get_pid_roll_rate(struct pid_params *out);
result_t params_get_pid_yaw_rate(struct pid_params *out);

result_t params_get_calib_acc_offset(float *out);
result_t params_get_calib_gyro_offset(float *out);
result_t params_get_calib_mag_scale(float *out);
result_t params_get_calib_mag_offset(float *out);

result_t params_get_triad_ref_acc(struct vec3 *out);
result_t params_get_triad_ref_mag(struct vec3 *out);

result_t params_get_madgwick_beta(float *out);

result_t params_get_limit(enum control_type type, struct control_limit *out);
result_t params_set_limit(enum control_type type, const struct control_limit *limit);

result_t params_get_pitch_mode(enum axis_mode *out);
result_t params_get_roll_mode(enum axis_mode *out);
result_t params_get_yaw_mode(enum axis_mode *out);

result_t params_get_motor_max(float *out);
result_t params_get_control_min_throttle(float *out);

result_t params_get_rcp_disc_time(uint32_t *out);

//-----------------------------------------------------------------

#endif /* PARAMS_H_ */
