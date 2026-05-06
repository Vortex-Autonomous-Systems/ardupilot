/*
  external control library for copter
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

class AP_ExternalControl_Copter : public AP_ExternalControl
{
public:
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
     */
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) override WARN_IF_UNUSED;

    /*
      Sets the target global position for a loiter point.
    */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;

    /*
      Set position, velocity, and acceleration target (NED frame).
      For guided mode full PVA control.
    */
    bool set_position_velocity_accel_NED(
        const Vector3p& pos_ned_m,
        const Vector3f& vel_ned_ms,
        const Vector3f& accel_ned_mss,
        bool use_yaw, float yaw_rad,
        bool use_yaw_rate, float yaw_rate_rads,
        bool yaw_is_relative,
        bool is_terrain_alt) override WARN_IF_UNUSED;

    /*
      Set attitude target for direct attitude control.
    */
    bool set_attitude_target(
        const Quaternion& att_quat,
        const Vector3f& ang_vel_body,
        float thrust_or_climb_rate,
        bool use_thrust) override WARN_IF_UNUSED;

private:
    /*
      Return true if Copter is ready to handle external control data.
      Currently checks mode and arm states.
    */
    bool ready_for_external_control() WARN_IF_UNUSED;
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
