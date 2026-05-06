# Direct attitude control message
# Bypasses position/velocity controllers for advanced applications

std_msgs/Header header

# Attitude quaternion (body-frame to reference frame)
geometry_msgs/Quaternion orientation

# Body-frame angular velocity [rad/s]
geometry_msgs/Vector3 body_rate

# Thrust or climb rate control
float32 thrust_normalized
float32 climb_rate_m_s
bool use_thrust

# Type mask for partial control
uint8 type_mask
uint8 IGNORE_ROLL_RATE = 1
uint8 IGNORE_PITCH_RATE = 2
uint8 IGNORE_YAW_RATE = 4
uint8 IGNORE_THRUST = 8
uint8 IGNORE_ATTITUDE = 64
