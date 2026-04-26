/*
  external control library for rover
 */


#include "AP_ExternalControl_Rover.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Rover.h"

/*
  set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw
  velocity is in earth frame, NED, m/s
*/
bool AP_ExternalControl_Rover::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads)
{
    return set_velocity_turn_rate_target(linear_velocity, yaw_rate_rads);
}

bool AP_ExternalControl_Rover::set_velocity_turn_rate_target(const Vector3f &linear_velocity, float yaw_rate_rads)
{
    if (!ready_for_external_control()) {
        return false;
    }

    // rover is commanded in body-frame using FRD convention
    auto &ahrs = AP::ahrs();
    Vector3f linear_velocity_body = ahrs.earth_to_body(linear_velocity);

    const float target_speed = linear_velocity_body.x;
    const float turn_rate_cds = isnan(yaw_rate_rads)? 0: degrees(yaw_rate_rads)*100;

    if (rover.control_mode->in_offboard_mode()) {
        rover.mode_offboard.set_desired_turn_rate_and_speed(turn_rate_cds, target_speed);
        return true;
    }

    rover.mode_guided.set_desired_turn_rate_and_speed(turn_rate_cds, target_speed);
    return true;
}

bool AP_ExternalControl_Rover::set_global_position(const Location& loc)
{
    return set_global_position_target(loc);
}

bool AP_ExternalControl_Rover::set_global_position_target(const Location& loc)
{
    if (!ready_for_external_control()) {
        return false;
    }

    if (rover.control_mode->in_offboard_mode()) {
        return rover.mode_offboard.set_desired_location(loc);
    }

    return rover.mode_guided.set_desired_location(loc);
}

bool AP_ExternalControl_Rover::set_offboard_heartbeat()
{
    if (!ready_for_offboard()) {
        return false;
    }
    rover.mode_offboard.set_heartbeat();
    return true;
}

bool AP_ExternalControl_Rover::set_offboard_stop()
{
    if (!ready_for_offboard()) {
        return false;
    }
    rover.mode_offboard.set_stop();
    return true;
}

bool AP_ExternalControl_Rover::set_offboard_hold()
{
    if (!ready_for_offboard()) {
        return false;
    }
    rover.mode_offboard.set_hold();
    return true;
}

bool AP_ExternalControl_Rover::get_offboard_status(OffboardStatus &status) const
{
    return rover.mode_offboard.get_status(status);
}

bool AP_ExternalControl_Rover::get_offboard_command_result(OffboardCommandResult &result) const
{
    return rover.mode_offboard.get_command_result(result);
}

bool AP_ExternalControl_Rover::get_offboard_tracking_status(OffboardTrackingStatus &tracking) const
{
    return rover.mode_offboard.get_tracking_status(tracking);
}

bool AP_ExternalControl_Rover::get_offboard_localization_status(OffboardLocalizationStatus &loc_status) const
{
    return rover.mode_offboard.get_localization_status(loc_status);
}

bool AP_ExternalControl_Rover::set_offboard_path(uint32_t path_id, float speed, const Location *points, uint8_t num_points)
{
    if (!ready_for_offboard()) {
        return false;
    }
    return rover.mode_offboard.set_path(path_id, speed, points, num_points);
}

bool AP_ExternalControl_Rover::set_offboard_trajectory(uint32_t traj_id, float speed, const Location *points, const float *speeds, uint8_t num_points)
{
    if (!ready_for_offboard()) {
        return false;
    }
    return rover.mode_offboard.set_trajectory(traj_id, speed, points, speeds, num_points);
}

bool AP_ExternalControl_Rover::set_offboard_pause()
{
    if (!ready_for_offboard()) {
        return false;
    }
    rover.mode_offboard.pause();
    return true;
}

bool AP_ExternalControl_Rover::set_offboard_resume()
{
    if (!ready_for_offboard()) {
        return false;
    }
    rover.mode_offboard.resume();
    return true;
}

bool AP_ExternalControl_Rover::set_offboard_abort()
{
    if (!ready_for_offboard()) {
        return false;
    }
    rover.mode_offboard.abort_task();
    return true;
}

bool AP_ExternalControl_Rover::set_offboard_safety_constraints(float max_speed, float min_distance)
{
    if (!ready_for_offboard()) {
        return false;
    }
    rover.mode_offboard.set_safety_constraints(max_speed, min_distance);
    return true;
}

bool AP_ExternalControl_Rover::set_offboard_dock()
{
    if (!ready_for_offboard()) {
        return false;
    }
    return rover.mode_offboard.start_dock();
}

bool AP_ExternalControl_Rover::ready_for_external_control()
{
    return (rover.control_mode->in_guided_mode() || ready_for_offboard()) && rover.arming.is_armed();
}

bool AP_ExternalControl_Rover::ready_for_offboard() const
{
    return rover.control_mode->in_offboard_mode() && rover.mode_offboard.enabled();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
