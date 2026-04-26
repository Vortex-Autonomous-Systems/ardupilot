#if AP_DDS_ENABLED

#include "AP_DDS_ExternalControl.h"
#include "AP_DDS_Frames.h"
#include <AP_AHRS/AP_AHRS.h>

#include <AP_ExternalControl/AP_ExternalControl.h>

// These are the Goal Interface constants. Because microxrceddsgen does not expose
// them in the generated code, they are manually maintained
// Position ignore flags
static constexpr uint16_t TYPE_MASK_IGNORE_LATITUDE = 1;
static constexpr uint16_t TYPE_MASK_IGNORE_LONGITUDE = 2;
static constexpr uint16_t TYPE_MASK_IGNORE_ALTITUDE = 4;

bool AP_DDS_External_Control::handle_global_position_control(ardupilot_msgs_msg_GlobalPosition& cmd_pos)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(cmd_pos.header.frame_id, MAP_FRAME) == 0) {
        // Narrow the altitude
        const int32_t alt_cm  = static_cast<int32_t>(cmd_pos.altitude * 100);

        Location::AltFrame alt_frame;
        if (!convert_alt_frame(cmd_pos.coordinate_frame, alt_frame)) {
            return false;
        }

        constexpr uint32_t MASK_POS_IGNORE =
            TYPE_MASK_IGNORE_LATITUDE |
            TYPE_MASK_IGNORE_LONGITUDE |
            TYPE_MASK_IGNORE_ALTITUDE;

        if (!(cmd_pos.type_mask & MASK_POS_IGNORE)) {
            Location loc(cmd_pos.latitude * 1E7, cmd_pos.longitude * 1E7, alt_cm, alt_frame);
            if (!external_control->set_global_position_target(loc)) {
                return false; // Don't try sending other commands if this fails
            }
        }

        // TODO add velocity and accel handling

        return true;
    }

    return false;
}

bool AP_DDS_External_Control::handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(cmd_vel.header.frame_id, BASE_LINK_FRAME_ID) == 0) {
        // Convert commands from body frame (x-forward, y-left, z-up) to NED.
        Vector3f linear_velocity;
        Vector3f linear_velocity_base_link {
            float(cmd_vel.twist.linear.x),
            float(cmd_vel.twist.linear.y),
            float(-cmd_vel.twist.linear.z) };
        const float yaw_rate = -cmd_vel.twist.angular.z;

        auto &ahrs = AP::ahrs();
        linear_velocity = ahrs.body_to_earth(linear_velocity_base_link);
        return external_control->set_velocity_turn_rate_target(linear_velocity, yaw_rate);
    }

    else if (strcmp(cmd_vel.header.frame_id, MAP_FRAME) == 0) {
        // Convert commands from ENU to NED frame
        Vector3f linear_velocity {
            float(cmd_vel.twist.linear.y),
            float(cmd_vel.twist.linear.x),
            float(-cmd_vel.twist.linear.z) };
        const float yaw_rate = -cmd_vel.twist.angular.z;
        return external_control->set_velocity_turn_rate_target(linear_velocity, yaw_rate);
    }

    return false;
}

bool AP_DDS_External_Control::handle_offboard_heartbeat(const ardupilot_msgs_msg_OffboardHeartbeat& heartbeat)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (!heartbeat.enabled) {
        return external_control->set_offboard_hold();
    }

    return external_control->set_offboard_heartbeat();
}

bool AP_DDS_External_Control::get_offboard_status(ardupilot_msgs_msg_OffboardStatus& status)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    AP_ExternalControl::OffboardStatus control_status {};
    if (!external_control->get_offboard_status(control_status)) {
        return false;
    }

    status.enabled = control_status.enabled;
    status.active = control_status.active;
    status.state = control_status.state;
    status.submode = control_status.submode;
    status.heartbeat_age_ms = control_status.heartbeat_age_ms;
    status.command_age_ms = control_status.command_age_ms;
    status.failsafe_action = control_status.failsafe_action;
    return true;
}

bool AP_DDS_External_Control::get_offboard_command_result(ardupilot_msgs_msg_OffboardCommandResult& result)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    AP_ExternalControl::OffboardCommandResult control_result {};
    if (!external_control->get_offboard_command_result(control_result)) {
        return false;
    }

    result.command_id = control_result.command_id;
    result.accepted = control_result.accepted;
    result.rejection_reason = control_result.rejection_reason;
    result.timestamp_ms = control_result.timestamp_ms;
    return true;
}

bool AP_DDS_External_Control::get_offboard_tracking_status(ardupilot_msgs_msg_OffboardTrackingStatus& tracking)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    AP_ExternalControl::OffboardTrackingStatus control_tracking {};
    if (!external_control->get_offboard_tracking_status(control_tracking)) {
        return false;
    }

    tracking.active_submode = control_tracking.active_submode;
    tracking.position_error = control_tracking.position_error;
    tracking.heading_error = control_tracking.heading_error;
    tracking.velocity_error = control_tracking.velocity_error;
    tracking.distance_remaining = control_tracking.distance_remaining;
    tracking.target_valid = control_tracking.target_valid;
    return true;
}

bool AP_DDS_External_Control::get_offboard_localization_status(ardupilot_msgs_msg_OffboardLocalizationStatus& loc_status)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    AP_ExternalControl::OffboardLocalizationStatus control_loc {};
    if (!external_control->get_offboard_localization_status(control_loc)) {
        return false;
    }

    loc_status.source_valid = control_loc.source_valid;
    loc_status.age_ms = control_loc.age_ms;
    loc_status.reset_count = control_loc.reset_count;
    loc_status.discontinuity_detected = control_loc.discontinuity_detected;
    loc_status.health = control_loc.health;
    return true;
}

bool AP_DDS_External_Control::handle_path_command(const ardupilot_msgs_msg_PathCommand& cmd)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    const uint8_t n = MIN(cmd.num_points, (uint8_t)16);
    if (n == 0) {
        return false;
    }

    Location points[16];
    for (uint8_t i = 0; i < n; i++) {
        points[i] = Location(cmd.latitudes[i] * 1E7, cmd.longitudes[i] * 1E7, 0, Location::AltFrame::ABOVE_HOME);
    }

    return external_control->set_offboard_path(cmd.path_id, cmd.speed, points, n);
}

bool AP_DDS_External_Control::handle_trajectory_command(const ardupilot_msgs_msg_TrajectoryCommand& cmd)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    const uint8_t n = MIN(cmd.num_points, (uint8_t)16);
    if (n == 0) {
        return false;
    }

    Location points[16];
    float speeds[16];
    for (uint8_t i = 0; i < n; i++) {
        points[i] = Location(cmd.latitudes[i] * 1E7, cmd.longitudes[i] * 1E7, 0, Location::AltFrame::ABOVE_HOME);
        speeds[i] = cmd.speeds[i];
    }

    return external_control->set_offboard_trajectory(cmd.trajectory_id, cmd.speed, points, speeds, n);
}

bool AP_DDS_External_Control::handle_safety_constraints(const ardupilot_msgs_msg_OffboardSafetyConstraints& msg)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }
    return external_control->set_offboard_safety_constraints(msg.max_speed, msg.min_distance);
}

bool AP_DDS_External_Control::convert_alt_frame(const uint8_t frame_in,  Location::AltFrame& frame_out)
{

    // Specified in ROS REP-147; only some are supported.
    switch (frame_in) {
    case 5: // FRAME_GLOBAL_INT
        frame_out = Location::AltFrame::ABSOLUTE;
        break;
    case 6: // FRAME_GLOBAL_REL_ALT
        frame_out = Location::AltFrame::ABOVE_HOME;
        break;
    case 11: // FRAME_GLOBAL_TERRAIN_ALT
        frame_out = Location::AltFrame::ABOVE_TERRAIN;
        break;
    default:
        return false;
    }
    return true;
}


#endif // AP_DDS_ENABLED
