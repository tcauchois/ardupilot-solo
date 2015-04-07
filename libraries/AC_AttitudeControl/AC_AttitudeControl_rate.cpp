// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl.h"

void AC_AttitudeControl::rate_controller_run()
{
    _motors.set_roll(rate_bf_to_motor_roll(_rate_bf_target.x));
    _motors.set_pitch(rate_bf_to_motor_pitch(_rate_bf_target.y));
    _motors.set_yaw(rate_bf_to_motor_yaw(_rate_bf_target.z));
}

float AC_AttitudeControl::rate_bf_to_motor_roll(float rate_target_rad)
{
    float error = degrees(rate_target_rad - _ahrs.get_gyro().x)*100.0f;

    _pid_rate_roll.set_input_filter_d(error);

    float p = _pid_rate_roll.get_p();
    float i = _pid_rate_roll.get_integrator();
    float d = _pid_rate_roll.get_d();

    if (!_motors.limit.roll_pitch || ((i>0&&error<0)||(i<0&&error>0))) {
        i = _pid_rate_roll.get_i();
    }

    return constrain_float(p+i+d,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
}

float AC_AttitudeControl::rate_bf_to_motor_pitch(float rate_target_rad)
{
    float error = degrees(rate_target_rad - _ahrs.get_gyro().y)*100.0f;

    _pid_rate_pitch.set_input_filter_d(error);

    float p = _pid_rate_pitch.get_p();
    float i = _pid_rate_pitch.get_integrator();
    float d = _pid_rate_pitch.get_d();

    if (!_motors.limit.roll_pitch || ((i>0&&error<0)||(i<0&&error>0))) {
        i = _pid_rate_pitch.get_i();
    }

    return constrain_float(p+i+d,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
}

float AC_AttitudeControl::rate_bf_to_motor_yaw(float rate_target_rad)
{
    float error = degrees(rate_target_rad - _ahrs.get_gyro().z)*100.0f;

    _pid_rate_yaw.set_input_filter_all(error);

    float p = _pid_rate_yaw.get_p();
    float i = _pid_rate_yaw.get_integrator();
    float d = _pid_rate_yaw.get_d();

    if (!_motors.limit.yaw || ((i>0&&error<0)||(i<0&&error>0))) {
        i = _pid_rate_yaw.get_i();
    }

    return constrain_float(p+i+d,-AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);
}
