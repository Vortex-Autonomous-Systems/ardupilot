#include "AP_DDS_config.h"

#if AP_DDS_ENABLED

#include "AP_DDS_ExternalControl.h"
#include "AP_DDS_Frames.h"
#include <AP_AHRS/AP_AHRS.h>

#include <AP_ExternalControl/AP_ExternalControl.h>

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
            GlobalPosition::IGNORE_LATITUDE |
            GlobalPosition::IGNORE_LONGITUDE |
            GlobalPosition::IGNORE_ALTITUDE;

        if (!(cmd_pos.type_mask & MASK_POS_IGNORE)) {
            Location loc(cmd_pos.latitude * 1E7, cmd_pos.longitude * 1E7, alt_cm, alt_frame);
            if (!external_control->set_global_position(loc)) {
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

        if (isnan(linear_velocity_base_link.y) && isnan(linear_velocity_base_link.z)) {
            // Assume it's an airspeed command so ignore the angular data.
            // While MAV_CMD_GUIDED_CHANGE_SPEED supports commands of ground speed and airspeed,
            // ROS users likely care more about airspeed control for a low level velocity control interface like this.
            return external_control->set_airspeed(linear_velocity_base_link.x);
        }

        const float yaw_rate = -cmd_vel.twist.angular.z;

        auto &ahrs = AP::ahrs();
        linear_velocity = ahrs.body_to_earth(linear_velocity_base_link);
        return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
    }

    else if (strcmp(cmd_vel.header.frame_id, MAP_FRAME) == 0) {
        // Convert commands from ENU to NED frame
        Vector3f linear_velocity {
            float(cmd_vel.twist.linear.y),
            float(cmd_vel.twist.linear.x),
            float(-cmd_vel.twist.linear.z) };
        const float yaw_rate = -cmd_vel.twist.angular.z;
        return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
    }

    return false;
}

bool AP_DDS_External_Control::arm(AP_Arming::Method method, bool do_arming_checks)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    return external_control->arm(method, do_arming_checks);
}

bool AP_DDS_External_Control::disarm(AP_Arming::Method method, bool do_disarm_checks)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    return external_control->disarm(method, do_disarm_checks);
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

#if AP_DDS_GUIDED_TARGET_SUB_ENABLED
bool AP_DDS_External_Control::handle_guided_target(ardupilot_msgs_msg_GuidedTarget& msg)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    // Convert from message frame to NED
    Vector3p pos_ned_m;
    Vector3f vel_ned_ms;
    Vector3f accel_ned_mss;
    float yaw_rad = msg.yaw_rad;
    float yaw_rate_rads = msg.yaw_rate_rad_s;

    // Frame constants from GuidedTarget message
    constexpr uint8_t FRAME_LOCAL_ENU = 2;

    if (msg.coordinate_frame == FRAME_LOCAL_ENU) {
        // ENU to NED conversion: swap X/Y, negate Z
        pos_ned_m = Vector3p(msg.position.y, msg.position.x, -msg.position.z);
        vel_ned_ms = Vector3f(msg.velocity.y, msg.velocity.x, -msg.velocity.z);
        accel_ned_mss = Vector3f(msg.acceleration.y, msg.acceleration.x, -msg.acceleration.z);
        // ENU yaw to NED yaw: negate
        yaw_rad = -yaw_rad;
        yaw_rate_rads = -yaw_rate_rads;
    } else {
        // Assume NED frame
        pos_ned_m = Vector3p(msg.position.x, msg.position.y, msg.position.z);
        vel_ned_ms = Vector3f(msg.velocity.x, msg.velocity.y, msg.velocity.z);
        accel_ned_mss = Vector3f(msg.acceleration.x, msg.acceleration.y, msg.acceleration.z);
    }

    // Zero out unused components based on flags
    if (!msg.use_position) {
        pos_ned_m.zero();
    }
    if (!msg.use_velocity) {
        vel_ned_ms.zero();
    }
    if (!msg.use_acceleration) {
        accel_ned_mss.zero();
    }

    return external_control->set_position_velocity_accel_NED(
        pos_ned_m, vel_ned_ms, accel_ned_mss,
        msg.use_yaw, yaw_rad,
        msg.use_yaw_rate, yaw_rate_rads,
        msg.yaw_is_relative,
        msg.is_terrain_relative);
}
#endif // AP_DDS_GUIDED_TARGET_SUB_ENABLED

#if AP_DDS_ATTITUDE_TARGET_SUB_ENABLED
bool AP_DDS_External_Control::handle_attitude_target(ardupilot_msgs_msg_AttitudeTarget& msg)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    // Convert quaternion from ROS (x, y, z, w) to ArduPilot Quaternion
    Quaternion att_quat(
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z
    );

    // Body-frame angular velocity
    Vector3f ang_vel_body(
        msg.body_rate.x,
        msg.body_rate.y,
        msg.body_rate.z
    );

    // Determine thrust or climb rate
    float thrust_or_climb_rate;
    if (msg.use_thrust) {
        thrust_or_climb_rate = msg.thrust_normalized;
    } else {
        thrust_or_climb_rate = msg.climb_rate_m_s;
    }

    return external_control->set_attitude_target(
        att_quat, ang_vel_body,
        thrust_or_climb_rate, msg.use_thrust);
}
#endif // AP_DDS_ATTITUDE_TARGET_SUB_ENABLED


#endif // AP_DDS_ENABLED