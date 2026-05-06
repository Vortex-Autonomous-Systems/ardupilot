# Unified guided mode setpoint message
# Supports position, velocity, acceleration, and yaw control

std_msgs/Header header

# Target type enumeration
uint8 target_type
uint8 TARGET_TYPE_POSITION = 0
uint8 TARGET_TYPE_VELOCITY = 1
uint8 TARGET_TYPE_POS_VEL = 2
uint8 TARGET_TYPE_POS_VEL_ACCEL = 3

# Coordinate frame (ROS REP-103 compatible)
uint8 coordinate_frame
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_LOCAL_ENU = 2
uint8 FRAME_BODY_NED = 3
uint8 FRAME_BODY_FRD = 4

# Position setpoint [m]
geometry_msgs/Point position
bool use_position

# Velocity setpoint [m/s]
geometry_msgs/Vector3 velocity
bool use_velocity

# Acceleration setpoint [m/s²]
geometry_msgs/Vector3 acceleration
bool use_acceleration

# Yaw control
float32 yaw_rad
float32 yaw_rate_rad_s
bool use_yaw
bool use_yaw_rate
bool yaw_is_relative

# Altitude handling
bool is_terrain_relative
