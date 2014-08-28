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
    PARAM_TYPE_FLOAT
};

struct param_info
{
    const char      *group;     // max group len: 14 chars + zero = 15
    const char      *name;      // max name len: 10 chars + zero = 11
    enum param_type type;
    uint8_t         size;
    uint8_t         count;
    void            *ptr;
};

//-----------------------------------------------------------------

result_t params_init(void);
void params_load_defaults(void);

result_t params_eeprom_save(void);
result_t params_eeprom_load(void);

result_t params_get_pid_pitch(struct pid_params *out);
result_t params_get_pid_roll(struct pid_params *out);
result_t params_get_pid_yaw(struct pid_params *out);
result_t params_get_pid_pitch_rate(struct pid_params *out);
result_t params_get_pid_roll_rate(struct pid_params *out);
result_t params_get_pid_yaw_rate(struct pid_params *out);

result_t params_get_mag_calib_scale(float *out);
result_t params_get_mag_calib_offset(float *out);

result_t params_get_triad_ref_acc(struct vec3 *out);
result_t params_get_triad_ref_mag(struct vec3 *out);

result_t params_get_madgwick_beta(float *out);

result_t params_get_throttle_limit(struct control_limit *out);
result_t params_get_angles_limits(struct control_limit_axes *out);
result_t params_get_rates_limits(struct control_limit_axes *out);

//-----------------------------------------------------------------

#endif /* PARAMS_H_ */
