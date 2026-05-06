/*
  external control library for MAVLink, DDS and scripting
 */

#pragma once

#include "AP_ExternalControl_config.h"

#if AP_EXTERNAL_CONTROL_ENABLED

#include <AP_Arming/AP_Arming.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

class AP_ExternalControl
{
public:

    AP_ExternalControl();

    /*
        Sets the target airspeed.
    */
    virtual bool set_airspeed(const float airspeed) WARN_IF_UNUSED {
        return false;
    }
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
     */
    virtual bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) WARN_IF_UNUSED {
        return false;
    }

    /*
        Sets the target global position with standard guided mode behavior.
    */
    virtual bool set_global_position(const Location& loc) WARN_IF_UNUSED {
        return false;
    }

    /*
        Arm the vehicle
    */
    virtual bool arm(AP_Arming::Method method, bool do_arming_checks) WARN_IF_UNUSED;

    /*
        Disarm the vehicle
    */
    virtual bool disarm(AP_Arming::Method method, bool do_disarm_checks) WARN_IF_UNUSED;

    /*
        Set position, velocity, and acceleration target (NED frame).
        For guided mode full PVA control.
    */
    virtual bool set_position_velocity_accel_NED(
        const Vector3p& pos_ned_m,
        const Vector3f& vel_ned_ms,
        const Vector3f& accel_ned_mss,
        bool use_yaw, float yaw_rad,
        bool use_yaw_rate, float yaw_rate_rads,
        bool yaw_is_relative,
        bool is_terrain_alt) WARN_IF_UNUSED {
        return false;
    }

    /*
        Set attitude target for direct attitude control.
        att_quat: Target attitude quaternion
        ang_vel_body: Body-frame angular velocity [rad/s]
        thrust_or_climb_rate: Thrust (-1 to 1) or climb rate (m/s)
        use_thrust: If true, use thrust; if false, use climb rate
    */
    virtual bool set_attitude_target(
        const Quaternion& att_quat,
        const Vector3f& ang_vel_body,
        float thrust_or_climb_rate,
        bool use_thrust) WARN_IF_UNUSED {
        return false;
    }

    static AP_ExternalControl *get_singleton(void) WARN_IF_UNUSED {
        return singleton;
    }
protected:
    ~AP_ExternalControl() {}

private:
    static AP_ExternalControl *singleton;
};


namespace AP
{
AP_ExternalControl *externalcontrol();
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
