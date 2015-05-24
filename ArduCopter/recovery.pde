/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

uint32_t motor_fail_start_time = 0;

void update_motor_fail_detector() {
    uint32_t tnow_ms = millis();
    if ( !motors.armed() || motors.motor_recovery_running() ) {
        motor_fail_start_time = tnow_ms;
        return;
    }

    bool motor_fail_criteria_met = false;
    uint8_t highest_motor = 0;

    // TODO logic

    if (motor_fail_criteria_met && tnow_ms-motor_fail_start_time > 250) {
        motors.do_motor_recovery((1<<highest_motor), 0.5f, 0.25f);
    } else {
        motor_fail_start_time = tnow_ms;
    }
}
