/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_OpticalFlow_PX4.cpp - ardupilot library for PX4Flow sensor
 *
 */

#include <AP_HAL.h>
#include "OpticalFlow.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_px4flow.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/uORB.h>

extern const AP_HAL::HAL& hal;

AP_OpticalFlow_PX4::AP_OpticalFlow_PX4(OpticalFlow &_frontend) : 
OpticalFlow_backend(_frontend) 
{}

void AP_OpticalFlow_PX4::init(void)
{
    // Try to open the FD first, then fall back to the orb publication
    _fd = open(PX4FLOW0_DEVICE_PATH, O_RDONLY);
    if (_fd != -1) {
        // change to 10Hz update
        if (ioctl(_fd, SENSORIOCSPOLLRATE, 10) != 0) {
            hal.console->printf("Unable to set flow rate to 10Hz\n");
        }
    } else {
        _orb_handle = orb_subscribe(ORB_ID(optical_flow));
    }
}

AP_OpticalFlow_PX4::~AP_OpticalFlow_PX4()
{
    if(_fd != -1)
        close(_fd);
    if(_orb_handle != -1)
        orb_unsubscribe(_orb_handle);
}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_PX4::update(void)
{
    /* Try the file handle, then fall back to the orb publication */
    while(1) {
        struct optical_flow_s report;
        if (_fd != -1) {
            if(::read(_fd, &report, sizeof(optical_flow_s)) == sizeof(optical_flow_s) &&
                report.timestamp != _last_timestamp) {}
            else break;
        }
        else if(_orb_handle != -1) {
            bool updated;
            if(orb_check(_orb_handle, &updated) != -1 && updated &&
                    orb_copy(ORB_ID(optical_flow), _orb_handle, &report) != -1) {}
            else break;
        }
        else return;
        struct OpticalFlow::OpticalFlow_state state;
        state.device_id = report.sensor_id;
        state.surface_quality = report.quality;
        if (report.integration_timespan > 0) {
            float yawAngleRad = _yawAngleRad();
            float cosYaw = cosf(yawAngleRad);
            float sinYaw = sinf(yawAngleRad);
            const Vector2f flowScaler = _flowScaler();
            float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
            float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
            float integralToRate = 1e6f / float(report.integration_timespan);
            // rotate sensor measurements from sensor to body frame through sensor yaw angle
            state.flowRate.x = flowScaleFactorX * integralToRate * (cosYaw * float(report.pixel_flow_x_integral) - sinYaw * float(report.pixel_flow_y_integral)); // rad/sec measured optically about the X body axis
            state.flowRate.y = flowScaleFactorY * integralToRate * (sinYaw * float(report.pixel_flow_x_integral) + cosYaw * float(report.pixel_flow_y_integral)); // rad/sec measured optically about the Y body axis
            state.bodyRate.x = integralToRate * (cosYaw * float(report.gyro_x_rate_integral) - sinYaw * float(report.gyro_y_rate_integral)); // rad/sec measured inertially about the X body axis
            state.bodyRate.y = integralToRate * (sinYaw * float(report.gyro_x_rate_integral) + cosYaw * float(report.gyro_y_rate_integral)); // rad/sec measured inertially about the Y body axis
        } else {
            state.flowRate.zero();
            state.bodyRate.zero();
        }
        _last_timestamp = report.timestamp;

        _update_frontend(state);
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
