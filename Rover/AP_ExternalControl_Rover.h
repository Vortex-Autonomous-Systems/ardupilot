/*
  external control library for rover
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

class AP_ExternalControl_Rover : public AP_ExternalControl {
public:
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
    */
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads)override WARN_IF_UNUSED;

    bool set_velocity_turn_rate_target(const Vector3f &linear_velocity, float yaw_rate_rads) override WARN_IF_UNUSED;

    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;

    bool set_global_position_target(const Location& loc) override WARN_IF_UNUSED;

    bool set_offboard_heartbeat() override WARN_IF_UNUSED;

    bool set_offboard_stop() override WARN_IF_UNUSED;

    bool set_offboard_hold() override WARN_IF_UNUSED;

    bool get_offboard_status(OffboardStatus &status) const override WARN_IF_UNUSED;

    bool get_offboard_command_result(OffboardCommandResult &result) const override WARN_IF_UNUSED;

    bool get_offboard_tracking_status(OffboardTrackingStatus &tracking) const override WARN_IF_UNUSED;

    bool get_offboard_localization_status(OffboardLocalizationStatus &loc_status) const override WARN_IF_UNUSED;

    bool set_offboard_path(uint32_t path_id, float speed, const Location *points, uint8_t num_points) override WARN_IF_UNUSED;

    bool set_offboard_trajectory(uint32_t traj_id, float speed, const Location *points, const float *speeds, uint8_t num_points) override WARN_IF_UNUSED;

    bool set_offboard_pause() override WARN_IF_UNUSED;

    bool set_offboard_resume() override WARN_IF_UNUSED;

    bool set_offboard_abort() override WARN_IF_UNUSED;

    bool set_offboard_safety_constraints(float max_speed, float min_distance) override WARN_IF_UNUSED;

    bool set_offboard_dock() override WARN_IF_UNUSED;
private:
    /*
      Return true if Rover is ready to handle external control data.
      Currently checks mode and arm states.
    */
    bool ready_for_external_control() WARN_IF_UNUSED;

    bool ready_for_offboard() const WARN_IF_UNUSED;
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
