#pragma once

#include "AP_DDS_config.h"

#if AP_DDS_ENABLED
#include "ardupilot_msgs/msg/GlobalPosition.h"
#include "geometry_msgs/msg/TwistStamped.h"
#if AP_DDS_GUIDED_TARGET_SUB_ENABLED
#include "ardupilot_msgs/msg/GuidedTarget.h"
#endif
#if AP_DDS_ATTITUDE_TARGET_SUB_ENABLED
#include "ardupilot_msgs/msg/AttitudeTarget.h"
#endif
#include <AP_Arming/AP_Arming.h>
#include <AP_Common/Location.h>

class AP_DDS_External_Control
{
public:
    // REP-147 Goal Interface Global Position Control
    // https://ros.org/reps/rep-0147.html#goal-interface
    static bool handle_global_position_control(ardupilot_msgs_msg_GlobalPosition& cmd_pos);
    static bool handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel);
    static bool arm(AP_Arming::Method method, bool do_arming_checks);
    static bool disarm(AP_Arming::Method method, bool do_disarm_checks);
    
#if AP_DDS_GUIDED_TARGET_SUB_ENABLED
    // Handle guided target subscription (ENU/NED position, velocity, acceleration)
    static bool handle_guided_target(ardupilot_msgs_msg_GuidedTarget& msg);
#endif
#if AP_DDS_ATTITUDE_TARGET_SUB_ENABLED
    // Handle attitude target subscription (quaternion, angular velocity)
    static bool handle_attitude_target(ardupilot_msgs_msg_AttitudeTarget& msg);
#endif

private:
    static bool convert_alt_frame(const uint8_t frame_in, Location::AltFrame& frame_out);
};
#endif // AP_DDS_ENABLED
