// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl.h"

// to be called by upper throttle controllers when they wish to provide throttle output directly to motors
void AC_AttitudeControl::set_throttle_out(float throttle_out, bool apply_angle_boost)
{
    _motors.set_stabilizing(true);
    if (apply_angle_boost) {
        _motors.set_throttle(get_boosted_throttle(throttle_out));
    }else{
        _motors.set_throttle(throttle_out);
        // clear angle_boost for logging purposes
        _angle_boost = 0;
    }
}

// outputs a throttle to all motors evenly with no attitude stabilization
void AC_AttitudeControl::set_throttle_out_unstabilized(float throttle_in, bool reset_att)
{
    if (reset_att) {
        reset_attitude_control();
    }
    _motors.set_stabilizing(false);
    _motors.set_throttle(throttle_in);
    _angle_boost = 0;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
float AC_AttitudeControl::get_boosted_throttle(float throttle_in)
{
    // inverted_factor is 1 for tilt angles below 60 degrees
    // reduces as a function of angle beyond 60 degrees
    // becomes zero at 90 degrees
    float min_throttle = _motors.throttle_min();
    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(2.0f*cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f/constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = (throttle_in-min_throttle)*inverted_factor*boost_factor + min_throttle;
    _angle_boost = throttle_out - throttle_in;
    return throttle_out;
}
