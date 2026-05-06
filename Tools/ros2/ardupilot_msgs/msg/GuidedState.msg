# Current guided mode state and targets

std_msgs/Header header

# Current guided sub-mode
uint8 sub_mode
uint8 SUB_MODE_TAKEOFF = 0
uint8 SUB_MODE_WP = 1
uint8 SUB_MODE_POS = 2
uint8 SUB_MODE_VEL = 3
uint8 SUB_MODE_POSVEL = 4
uint8 SUB_MODE_POSVELACCEL = 5
uint8 SUB_MODE_ANGLE = 6

# Current targets (NED frame, converted from internal state)
geometry_msgs/Point target_position_ned_m
geometry_msgs/Vector3 target_velocity_ned_ms
geometry_msgs/Vector3 target_accel_ned_mss
float32 target_yaw_rad

# Status
bool is_paused
bool has_valid_target
float32 wp_distance_m
float32 wp_bearing_deg
