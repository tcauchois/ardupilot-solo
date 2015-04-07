// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl.h"

void AC_AttitudeControl::constrain_quaternion_tilt(Quaternion& q, float tilt_limit_rad) {
    // rotate a unit down vector into earth frame
    Matrix3f mat;
    q.rotation_matrix(mat);
    Vector3f down_direction_vec = mat.mul_transpose(Vector3f(0.0, 0.0, 1.0));

    float tilt_mag = atan2f(sqrtf(sq(down_direction_vec.x)+sq(down_direction_vec.y)), down_direction_vec.z);

    if (tilt_mag <= tilt_limit_rad) {
        return;
    }

    // finish computing a body-frame tilt vector and its norm
    Vector3f tilt_vec(down_direction_vec.y,-down_direction_vec.x, 0.0f);
    float tilt_vec_mag = tilt_vec.length();
    Vector3f tilt_vec_norm = tilt_vec;
    if (tilt_vec_mag == 0.0f) {
        tilt_vec_norm.x = 1.0f;
        tilt_vec_norm.y = 0.0f;
    } else {
        tilt_vec_norm /= tilt_vec_mag;
    }
    tilt_vec = tilt_vec_norm * tilt_mag;
    tilt_vec_mag = tilt_mag;

    // rotate the quaternion up along the tilt vector such that its tilt lies on the limit circle
    q.rotate(tilt_vec_norm*tilt_limit_rad-tilt_vec);
}

void AC_AttitudeControl::get_att_p_vec(Vector3f& ret) {
    ret.x = _p_angle_roll.kP();
    ret.y = _p_angle_pitch.kP();
    ret.z = _p_angle_yaw.kP();
}

void AC_AttitudeControl::get_accel_lim_vec(Vector3f& ret) {
    ret.x = radians(_accel_roll_max/100.0f);
    ret.y = radians(_accel_pitch_max/100.0f);
    ret.z = radians(_accel_yaw_max/100.0f);
}

float AC_AttitudeControl::ellipsoid_radius_in_direction(const Vector3f& ellipsoid_axes, const Vector3f& dir) {
    return (ellipsoid_axes.x*ellipsoid_axes.y*ellipsoid_axes.z)/sqrtf(sq(ellipsoid_axes.y)*(sq(ellipsoid_axes.x)*sq(dir.z)+sq(ellipsoid_axes.z)*sq(dir.x))+sq(ellipsoid_axes.x)*sq(ellipsoid_axes.z)*sq(dir.y));
}

void AC_AttitudeControl::update_angle_ef_target() {
    _att_target.to_euler(_angle_ef_target.x,_angle_ef_target.y,_angle_ef_target.z);
}

void AC_AttitudeControl::attitude_error(const Quaternion& setpoint, const Quaternion& process_variable, Vector3f &ret)
{
    (process_variable.inverse()*setpoint).to_axis_angle(ret);
}

void AC_AttitudeControl::vector_sqrt_controller(const Vector3f& error, const Vector3f& p_vec, const Vector3f& accel_lim_vec, Vector3f &ret)
{
    if (accel_lim_vec.x == 0.0f || accel_lim_vec.y == 0.0f || accel_lim_vec.z == 0.0f) {
        // do the axes separately because the ellipsoid is undefined
        // output is not necessarily in the same direction as error
        // accel limit ellipsoid is not necessarily respected
        ret.x = sqrt_controller(error.x,p_vec.x,accel_lim_vec.x);
        ret.y = sqrt_controller(error.y,p_vec.y,accel_lim_vec.y);
        ret.z = sqrt_controller(error.z,p_vec.z,accel_lim_vec.z);
    } else {
        // do a 3-axis sqrt controller
        // output is always in the same direction as error
        // accel limit ellipsoid is always respected
        float error_mag = error.length();

        if(error_mag == 0.0f) {
            ret.zero();
            return;
        }

        Vector3f error_norm = error/error_mag;

        float p_in_dir = ellipsoid_radius_in_direction(p_vec, error_norm);

        float accel_lim_in_dir = ellipsoid_radius_in_direction(accel_lim_vec, error_norm);

        float rate_target = sqrt_controller(error_mag, p_in_dir, accel_lim_in_dir);

        ret = error_norm * rate_target;

        // check output for nan and inf
        if(ret.is_nan() || ret.is_inf()) {
            ret.zero();
        }
    }
}

void AC_AttitudeControl::frame_conversion_ef_to_bf(const Vector3f& ef_vector, Vector3f& bf_vector)
{
    bf_vector.x = ef_vector.x - _ahrs.sin_pitch() * ef_vector.z;
    bf_vector.y = _ahrs.cos_roll()  * ef_vector.y + _ahrs.sin_roll() * _ahrs.cos_pitch() * ef_vector.z;
    bf_vector.z = -_ahrs.sin_roll() * ef_vector.y + _ahrs.cos_pitch() * _ahrs.cos_roll() * ef_vector.z;
}

float AC_AttitudeControl::sqrt_controller(float error, float p, float second_ord_lim)
{
    if (second_ord_lim == 0.0f || p == 0.0f) {
        return error*p;
    }

    float linear_dist = second_ord_lim/sq(p);

    if (error > linear_dist) {
        return safe_sqrt(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
    } else if (error < -linear_dist) {
        return -safe_sqrt(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
    } else {
        return error*p;
    }
}

float AC_AttitudeControl::inverse_sqrt_controller(float output, float p, float second_ord_lim)
{
    if (second_ord_lim == 0.0f || p == 0.0f) {
        return output/p;
    }

    float linear_dist = second_ord_lim/(p);

    if (output > linear_dist) {
        return (sq(second_ord_lim) + sq(output) * sq(p)) / (2*second_ord_lim*sq(p));
    } else if (output < -linear_dist) {
        return -(sq(second_ord_lim) + sq(output) * sq(p)) / (2*second_ord_lim*sq(p));
    } else {
        return output/p;
    }
}

// Maximum roll rate step size that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float alpha = _pid_rate_roll.get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*_pid_rate_roll.kD())/_dt + _pid_rate_roll.kP());
}

// Maximum pitch rate step size that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    float alpha = _pid_rate_pitch.get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*_pid_rate_pitch.kD())/_dt + _pid_rate_pitch.kP());
}

// Maximum yaw rate step size that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    float alpha = _pid_rate_yaw.get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*_pid_rate_yaw.kD())/_dt + _pid_rate_yaw.kP());
}
