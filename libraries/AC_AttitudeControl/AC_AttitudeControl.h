// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl.h
/// @brief   ArduCopter attitude control library

#ifndef AC_AttitudeControl_H
#define AC_AttitudeControl_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_InertialSensor.h>
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <AC_PID.h>
#include <AC_P.h>

// only kept for heli:
#define AC_ATTITUDE_CONTROL_DEGX100                  5729.57795f // constant to convert from radians to centi-degrees
#define AC_ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT            18000 // maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
#define AC_ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT              9000 // maximum rotation rate on yaw axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes

// parameter default values
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT                1000 // constraint on yaw angle error in degrees.  This should lead to maximum turn rate of 10deg/sed * Stab Rate P so by default will be 45deg/sec.
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT               0 // default maximum acceleration for roll/pitch axis in centi-degrees/sec/sec
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT                0 // default maximum acceleration for yaw axis in centi-degrees/sec/sec
#define AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT                 0  // body-frame rate feedforward enabled by default

// maximum output values
#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX            5000.0f // body-frame rate controller maximum output (for roll-pitch axis)
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX           4500.0f // body-frame rate controller maximum output (for yaw axis)

// maximum overshoot angles
#define AC_ATTITUDE_RATE_STAB_RP_OVERSHOOT_ANGLE_MAX_DEG   300.0f // earth-frame rate stabilize controller's maximum overshoot angle (never limited)
#define AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_DEG   10.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_DEG  30.0f // earth-frame rate stabilize controller's maximum overshoot angle

// rate options
#define AC_ATTITUDE_100HZ_DT                              0.0100f // delta time in seconds for 100hz update rate
#define AC_ATTITUDE_400HZ_DT                              0.0025f // delta time in seconds for 400hz update rate


class AC_AttitudeControl {
public:
    AC_AttitudeControl( AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_Motors& motors,
                        AC_P& pi_angle_roll, AC_P& pi_angle_pitch, AC_P& pi_angle_yaw,
                        AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw
                        ) :
        _ahrs(ahrs),
        _aparm(aparm),
        _motors(motors),
        _p_angle_roll(pi_angle_roll),
        _p_angle_pitch(pi_angle_pitch),
        _p_angle_yaw(pi_angle_yaw),
        _pid_rate_roll(pid_rate_roll),
        _pid_rate_pitch(pid_rate_pitch),
        _pid_rate_yaw(pid_rate_yaw),
        _dt(AC_ATTITUDE_100HZ_DT),
        _angle_boost(0),
        _acro_angle_switch(0)
        {
            AP_Param::setup_object_defaults(this, var_info);

            // initialise flags
            _flags.limit_angle_to_rate_request = true;
        }

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl() {}

    // getters
    int16_t lean_angle_max() const { return _aparm.angle_max; }
    bool get_bf_feedforward() { return _rate_bf_ff_enabled; }
    Vector3f rate_bf_targets() const { return _rate_bf_target*degrees(1.0f)*100.0f; }
    int16_t angle_boost() const { return _angle_boost; }
    Vector3f angle_ef_targets() const { return _angle_ef_target*degrees(1.0f)*100.0f; }

// attitude control and public interface - defined in AC_AttitudeControl.cpp
public:
    void angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain);
    void angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef);
    void angle_ef_roll_pitch_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw);
    void rate_ef_roll_pitch_yaw(float roll_rate_ef, float pitch_rate_ef, float yaw_rate_ef);
    void rate_bf_roll_pitch_yaw(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf);

    void rate_bf_roll_target(float rate_cds) { _rate_bf_target.x = rate_cds; }
    void rate_bf_pitch_target(float rate_cds) { _rate_bf_target.y = rate_cds; }
    void rate_bf_yaw_target(float rate_cds) { _rate_bf_target.z = rate_cds; }

    void reset_attitude_control();

protected:
    void _rate_bf_roll_pitch_yaw(Vector3f rate_rads, bool constrain_tilt);
    void attitude_controller_run();
    void calc_constrained_angle_error_and_target();

// rate control - defined in AC_AttitudeControl_rate.cpp
public:
    virtual void rate_controller_run();

protected:
    float rate_bf_to_motor_roll(float rate_target_cds);
    float rate_bf_to_motor_pitch(float rate_target_cds);
    virtual float rate_bf_to_motor_yaw(float rate_target_cds);

// config - defined in AC_AttitudeControl_config.cpp
public:
    static const struct AP_Param::GroupInfo var_info[];
    void set_dt(float delta_sec);
    void accel_limiting(bool enable_or_disable);
    void bf_feedforward(bool enable_or_disable) { _rate_bf_ff_enabled = enable_or_disable; }
    void limit_angle_to_rate_request(bool limit_request) { _flags.limit_angle_to_rate_request = limit_request; }
    void save_accel_roll_max(float accel_roll_max) { _accel_roll_max = accel_roll_max; _accel_roll_max.save(); }
    void save_accel_pitch_max(float accel_pitch_max) { _accel_pitch_max = accel_pitch_max; _accel_pitch_max.save(); }
    void save_accel_yaw_max(float accel_yaw_max) { _accel_yaw_max = accel_yaw_max; _accel_yaw_max.save(); }


// throttle - defined in AC_AttitudeControl_throttle.cpp
public:
    void set_throttle_out(float throttle_pwm, bool apply_angle_boost);
    void set_throttle_out_unstabilized(float throttle_in, bool reset_attitude_control);

protected:
    virtual float get_boosted_throttle(float throttle_in);

// helpers - defined in AC_AttitudeControl_helpers.cpp
public:
    void frame_conversion_ef_to_bf(const Vector3f& ef_vector, Vector3f& bf_vector);
    static float sqrt_controller(float error, float p, float second_ord_lim);
    static float inverse_sqrt_controller(float output, float p, float second_ord_lim);
    float max_rate_step_bf_roll();
    float max_rate_step_bf_pitch();
    float max_rate_step_bf_yaw();
    float max_angle_step_bf_roll() { return max_rate_step_bf_roll()/_p_angle_roll.kP(); }
    float max_angle_step_bf_pitch() { return max_rate_step_bf_pitch()/_p_angle_pitch.kP(); }
    float max_angle_step_bf_yaw() { return max_rate_step_bf_yaw()/_p_angle_yaw.kP(); }

protected:
    void constrain_quaternion_tilt(Quaternion& q, float tilt_limit_rad);
    void get_att_p_vec(Vector3f& ret);
    void get_accel_lim_vec(Vector3f& ret);
    float ellipsoid_radius_in_direction(const Vector3f& ellipsoid_axes, const Vector3f& dir);
    void update_angle_ef_target();
    void attitude_error(const Quaternion& setpoint, const Quaternion& process_variable, Vector3f &ret);
    void vector_sqrt_controller(const Vector3f& error, const Vector3f& p_vec, const Vector3f& accel_lim_vec, Vector3f &ret);

// member variable declarations
protected:
    struct AttControlFlags {
        uint8_t limit_angle_to_rate_request :   1;  // 1 if the earth frame angle controller is limiting it's maximum rate request
        uint8_t constrain_angle_error : 1; // 1 if the controller is in an acro mode
    } _flags;


    // references to external libraries
    const AP_AHRS&      _ahrs;
    const AP_Vehicle::MultiCopter &_aparm;
    AP_Motors&          _motors;
    AC_P&	            _p_angle_roll;
    AC_P&	            _p_angle_pitch;
    AC_P&	            _p_angle_yaw;
    AC_PID&             _pid_rate_roll;
    AC_PID&             _pid_rate_pitch;
    AC_PID&             _pid_rate_yaw;

    // parameters
    AP_Float            _angle_rate_rp_max;     // maximum rate request output from the earth-frame angle controller for roll and pitch axis
    AP_Float            _angle_rate_y_max;      // maximum rate request output from the earth-frame angle controller for yaw axis
    AP_Float            _slew_yaw;              // maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    AP_Float            _accel_roll_max;          // maximum rotation acceleration for earth-frame roll axis
    AP_Float            _accel_pitch_max;          // maximum rotation acceleration for earth-frame pitch axis
    AP_Float            _accel_yaw_max;           // maximum rotation acceleration for earth-frame yaw axis
    AP_Int8             _rate_bf_ff_enabled;    // Enable/Disable body frame rate feed forward

    // internal variables
    float               _dt;                    // time delta in seconds
    Quaternion          _att_target;            // attitude target quaternion
    Vector3f            _angle_ef_target;       // attitude target euler angles (kept consistent with _att_target)
    Vector3f            _angle_bf_error;        // angle controller body-frame error - angle difference between _att_target and copter attitude in copter attitude frame
    Vector3f            _rate_bf_target;        // rate controller body-frame targets
    Vector3f            _rate_bf_desired;       // body-frame feed forward rates
    int16_t             _angle_boost;           // used only for logging
    int16_t             _acro_angle_switch;           // used only for logging
};

#define AC_ATTITUDE_CONTROL_LOG_FORMAT(msg) { msg, sizeof(AC_AttitudeControl::log_Attitude),	\
                            "ATT", "cccccCC",      "RollIn,Roll,PitchIn,Pitch,YawIn,Yaw,NavYaw" }

#endif //AC_AttitudeControl_H
