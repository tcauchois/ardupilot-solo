// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl.h"
#include <AP_HAL.h>
#include <stdio.h>

void AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain)
{
    _flags.constrain_angle_error = false;

    // convert from centidegrees to radians
    roll_angle_ef = radians(roll_angle_ef/100.0f);
    pitch_angle_ef = radians(pitch_angle_ef/100.0f);
    yaw_rate_ef = radians(yaw_rate_ef/100.0f);

    // constrain smoothing gain
    smoothing_gain = constrain_float(smoothing_gain,1.0f,50.0f);

    // get accel limits as a vector
    Vector3f accel_lim_vec;
    get_accel_lim_vec(accel_lim_vec);

    // ensure that enough control authority is left for yaw
    // TODO parameters need to be reasonably constrained so that these x and y accel limits don't hit zero
    float angle_max = radians(_aparm.angle_max/100.0f);
    float yaw_xy_headroom = sinf(angle_max)*accel_lim_vec.z;
    accel_lim_vec.x -= yaw_xy_headroom;
    accel_lim_vec.y -= yaw_xy_headroom;

    // compute a new target quaternion
    Quaternion new_target;
    new_target.from_euler(roll_angle_ef, pitch_angle_ef, _angle_ef_target.z);
    constrain_quaternion_tilt(new_target, angle_max);

    // get angle difference between the new target quaternion and the old target quaternion
    Vector3f angle_diff;
    attitude_error(new_target, _att_target, angle_diff);

    // compute new desired rate vector
    Vector3f tilt_rate_bf;
    const Vector3f smoothing_gain_vec(smoothing_gain,smoothing_gain,smoothing_gain);
    vector_sqrt_controller(angle_diff, smoothing_gain_vec, accel_lim_vec, tilt_rate_bf);

    // constrain yaw acceleration in the earth frame
    // not strictly necessary - this prevents the earth-frame yaw response from
    // changing when the copter is tilted over
    Matrix3f mat;
    _att_target.rotation_matrix(mat);
    Vector3f rate_ef_desired = mat * _rate_bf_desired;
    float yaw_rate_change_limit = accel_lim_vec.z * _dt;
    yaw_rate_ef = constrain_float(yaw_rate_ef, rate_ef_desired.z-yaw_rate_change_limit, rate_ef_desired.z+yaw_rate_change_limit);

    // add yaw rate
    Vector3f yaw_rate_bf = mat.mul_transpose(Vector3f(0.0f, 0.0f, yaw_rate_ef));

    // use the acro controller to output the computed rate
    _rate_bf_roll_pitch_yaw(tilt_rate_bf+yaw_rate_bf, true);
}

void AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef)
{
    _flags.constrain_angle_error = false;

    // TODO fix up this hack a bit
    // do we really want to turn off _rate_bf_ff?
    int8_t prev_rate_bf_ff_enab = _rate_bf_ff_enabled;
    _rate_bf_ff_enabled = false;
    angle_ef_roll_pitch_rate_ef_yaw_smooth(roll_angle_ef, pitch_angle_ef, yaw_rate_ef, 50.0f);
    _rate_bf_ff_enabled = prev_rate_bf_ff_enab;
}

void AC_AttitudeControl::angle_ef_roll_pitch_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw)
{
    _flags.constrain_angle_error = false;

    attitude_controller_run();

    // convert from centidegrees to radians
    roll_angle_ef = radians(roll_angle_ef/100.0f);
    pitch_angle_ef = radians(pitch_angle_ef/100.0f);
    yaw_angle_ef = radians(yaw_angle_ef/100.0f);

    _angle_ef_target.x = roll_angle_ef;
    _angle_ef_target.y = pitch_angle_ef;
    if (slew_yaw) {
        float yaw_change_limit = radians(_slew_yaw/100.0f)*_dt;
        _angle_ef_target.z += constrain_float(wrap_PI(yaw_angle_ef-_angle_ef_target.z),-yaw_change_limit,yaw_change_limit);
    } else {
        _angle_ef_target.z = yaw_angle_ef;
    }

    _att_target.from_euler(_angle_ef_target.x, _angle_ef_target.y, _angle_ef_target.z);
    constrain_quaternion_tilt(_att_target, radians(_aparm.angle_max/100.0f));

    _rate_bf_desired.zero();

    update_angle_ef_target();
}

void AC_AttitudeControl::rate_ef_roll_pitch_yaw(float roll_rate_ef, float pitch_rate_ef, float yaw_rate_ef)
{
    _flags.constrain_angle_error = true;

    // convert from centidegrees to radians
    roll_rate_ef = radians(roll_rate_ef/100.0f);
    pitch_rate_ef = radians(pitch_rate_ef/100.0f);
    yaw_rate_ef = radians(yaw_rate_ef/100.0f);

    Vector3f rate_target_bf = Vector3f(roll_rate_ef, pitch_rate_ef, yaw_rate_ef);
    frame_conversion_ef_to_bf(rate_target_bf, rate_target_bf);

    _rate_bf_roll_pitch_yaw(rate_target_bf, true);
}

void AC_AttitudeControl::rate_bf_roll_pitch_yaw(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf)
{
    _flags.constrain_angle_error = true;

    // convert from centidegrees per second to radians per second
    roll_rate_bf = radians(roll_rate_bf/100.0f);
    pitch_rate_bf = radians(pitch_rate_bf/100.0f);
    yaw_rate_bf = radians(yaw_rate_bf/100.0f);

    _rate_bf_roll_pitch_yaw(Vector3f(roll_rate_bf,pitch_rate_bf,yaw_rate_bf), false);
}

void AC_AttitudeControl::_rate_bf_roll_pitch_yaw(Vector3f bf_rotation_rate, bool constrain_tilt)
{
    attitude_controller_run();

    // get accel limits as a vector
    Vector3f accel_lim_vec;
    get_accel_lim_vec(accel_lim_vec);

    // take the vector difference between new desired rate and old desired rate
    Vector3f rate_bf_desired_increment = bf_rotation_rate-_rate_bf_desired;
    float rate_bf_desired_increment_mag = rate_bf_desired_increment.length();

    bool no_feedforward = ((accel_lim_vec.x == 0.0f && accel_lim_vec.y == 0.0f) || !_rate_bf_ff_enabled) && accel_lim_vec.z != 0.0f;
    bool do_3_axis_feedforward = _rate_bf_ff_enabled && accel_lim_vec.x != 0.0f && accel_lim_vec.y != 0.0f && accel_lim_vec.z != 0.0f;

    if (no_feedforward) {
        _rate_bf_desired.zero();
    } else if(!do_3_axis_feedforward) {
        if (accel_lim_vec.x != 0.0f && _rate_bf_ff_enabled) {
            float rate_change_limit = accel_lim_vec.x * _dt;
            _rate_bf_desired.x += constrain_float(rate_bf_desired_increment.x, -rate_change_limit, rate_change_limit);
            bf_rotation_rate.x = _rate_bf_desired.x;
        } else {
            _rate_bf_desired.x = 0.0f;
        }
        if (accel_lim_vec.y != 0.0f && _rate_bf_ff_enabled) {
            float rate_change_limit = accel_lim_vec.y * _dt;
            _rate_bf_desired.y += constrain_float(rate_bf_desired_increment.y, -rate_change_limit, rate_change_limit);
            bf_rotation_rate.y = _rate_bf_desired.y;
        } else {
            _rate_bf_desired.y = 0.0f;
        }
        if (accel_lim_vec.z != 0.0f) {
            float rate_change_limit = accel_lim_vec.z * _dt;
            _rate_bf_desired.z += constrain_float(rate_bf_desired_increment.z, -rate_change_limit, rate_change_limit);
            bf_rotation_rate.z = _rate_bf_desired.z;
        } else {
            _rate_bf_desired.z = 0.0f;
        }
    } else if (rate_bf_desired_increment_mag > 0.0f) {
        Vector3f rate_bf_desired_increment_norm = rate_bf_desired_increment/rate_bf_desired_increment_mag;

        // find angular acceleration limit about axis of angular acceleration
        float accel_lim_about_axis = ellipsoid_radius_in_direction(accel_lim_vec, rate_bf_desired_increment_norm);

        // limit the rate of change of _rate_bf_desired
        float rate_change_limit = accel_lim_about_axis * _dt;
        if (rate_bf_desired_increment_mag > rate_change_limit) {
            _rate_bf_desired += rate_bf_desired_increment_norm*rate_change_limit;
        } else {
            _rate_bf_desired += rate_bf_desired_increment;
        }
        bf_rotation_rate = _rate_bf_desired;
    }

    // update the attitude target
    _att_target.rotate(bf_rotation_rate * _dt);
    if (constrain_tilt) {
        constrain_quaternion_tilt(_att_target, radians(_aparm.angle_max/100.0f));
    }

    update_angle_ef_target();
}

void AC_AttitudeControl::calc_constrained_angle_error_and_target()
{
    static const float acro_overshoot_max = radians(AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_DEG);
    static const float rp_overshoot_max = radians(AC_ATTITUDE_RATE_STAB_RP_OVERSHOOT_ANGLE_MAX_DEG);
    static const float yaw_overshoot_max = radians(AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_DEG);
    static const Vector3f overshoot_max_vec(rp_overshoot_max,rp_overshoot_max,yaw_overshoot_max);

    // get the copter attitude
    Quaternion copter_att;
    copter_att.from_rotation_matrix(_ahrs.get_dcm_matrix());

    // take the body-frame angle error
    attitude_error(_att_target, copter_att, _angle_bf_error);

    float angle_bf_error_mag = _angle_bf_error.length();
    if (angle_bf_error_mag == 0.0f) {
        return;
    }
    Vector3f angle_bf_error_norm = _angle_bf_error/angle_bf_error_mag;

    float overshoot_limit;
    if(_flags.constrain_angle_error) {
        overshoot_limit = acro_overshoot_max;
    } else {
        overshoot_limit = ellipsoid_radius_in_direction(overshoot_max_vec, angle_bf_error_norm);
    }

    if (angle_bf_error_mag <= overshoot_limit) {
        return;
    }

    _angle_bf_error *= overshoot_limit/angle_bf_error_mag;

    _att_target = copter_att;
    _att_target.rotate(_angle_bf_error);
}

void AC_AttitudeControl::attitude_controller_run()
{
    // calculate and constrain angle error and angle target
    calc_constrained_angle_error_and_target();

    // normalize the attitude target
    _att_target.normalize();

    // get acceleration limit vector
    Vector3f accel_lim_vec;
    get_accel_lim_vec(accel_lim_vec);

    // get stab p term vector
    Vector3f att_p_vec;
    get_att_p_vec(att_p_vec);

    vector_sqrt_controller(_angle_bf_error, att_p_vec, accel_lim_vec, _rate_bf_target);
    if (_rate_bf_ff_enabled) {
        _rate_bf_target += _rate_bf_desired;
    } else {
        _rate_bf_target.z += _rate_bf_desired.z;
    }

    _rate_bf_target.x += _angle_bf_error.y * _ahrs.get_gyro().z;
    _rate_bf_target.y += -_angle_bf_error.x * _ahrs.get_gyro().z;

    update_angle_ef_target();
}

void AC_AttitudeControl::reset_attitude_control()
{
    _pid_rate_roll.reset_I();
    _pid_rate_pitch.reset_I();
    _pid_rate_yaw.reset_I();

    _att_target.from_rotation_matrix(_ahrs.get_dcm_matrix());
    update_angle_ef_target();
    _rate_bf_desired.zero();
    _rate_bf_target = _ahrs.get_gyro();
    _angle_bf_error.zero();
}
