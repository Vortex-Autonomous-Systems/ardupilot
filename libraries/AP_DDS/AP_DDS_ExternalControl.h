#pragma once

#if AP_DDS_ENABLED
#include "ardupilot_msgs/msg/OffboardHeartbeat.h"
#include "ardupilot_msgs/msg/OffboardStatus.h"
#include "ardupilot_msgs/msg/OffboardCommandResult.h"
#include "ardupilot_msgs/msg/OffboardTrackingStatus.h"
#include "ardupilot_msgs/msg/OffboardLocalizationStatus.h"
#include "ardupilot_msgs/msg/PathCommand.h"
#include "ardupilot_msgs/msg/TrajectoryCommand.h"
#include "ardupilot_msgs/msg/OffboardSafetyConstraints.h"
#include "ardupilot_msgs/msg/GlobalPosition.h"
#include "geometry_msgs/msg/TwistStamped.h"

#include <AP_Common/Location.h>

class AP_DDS_External_Control
{
public:
    // REP-147 Goal Interface Global Position Control
    // https://ros.org/reps/rep-0147.html#goal-interface
    static bool handle_global_position_control(ardupilot_msgs_msg_GlobalPosition& cmd_pos);
    static bool handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel);
    static bool handle_offboard_heartbeat(const ardupilot_msgs_msg_OffboardHeartbeat& heartbeat);
    static bool get_offboard_status(ardupilot_msgs_msg_OffboardStatus& status);
    static bool get_offboard_command_result(ardupilot_msgs_msg_OffboardCommandResult& result);
    static bool get_offboard_tracking_status(ardupilot_msgs_msg_OffboardTrackingStatus& tracking);
    static bool get_offboard_localization_status(ardupilot_msgs_msg_OffboardLocalizationStatus& loc_status);
    static bool handle_path_command(const ardupilot_msgs_msg_PathCommand& cmd);
    static bool handle_trajectory_command(const ardupilot_msgs_msg_TrajectoryCommand& cmd);
    static bool handle_safety_constraints(const ardupilot_msgs_msg_OffboardSafetyConstraints& msg);
private:
    static bool convert_alt_frame(const uint8_t frame_in,  Location::AltFrame& frame_out);
};
#endif // AP_DDS_ENABLED
