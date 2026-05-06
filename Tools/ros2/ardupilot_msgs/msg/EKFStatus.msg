# EKF Status message for DDS transport
# EKF health, innovation variances and filter status

std_msgs/Header header

uint8 ekf_type              # EKF type (2=EKF2, 3=EKF3)
uint8 active_lane           # Active EKF lane
bool is_healthy             # Overall EKF health
bool attitude_ok            # Attitude estimate good
bool velocity_horiz_ok      # Horizontal velocity estimate good
bool velocity_vert_ok       # Vertical velocity estimate good
bool position_horiz_ok      # Horizontal position estimate good
bool position_vert_ok       # Vertical position estimate good
bool compass_ok             # Compass health
bool terrain_ok             # Terrain altitude estimate good

# Variance values (lower is better)
float32 velocity_variance   # Velocity innovation variance
float32 pos_horiz_variance  # Horizontal position variance
float32 pos_vert_variance   # Vertical position variance
float32 compass_variance    # Compass variance
float32 terrain_variance    # Terrain variance
