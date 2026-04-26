# Time-stamped trajectory segment for DDS OFFBOARD mode.
# Each point carries a timestamp offset, position, and velocity.
# The vehicle interpolates between points for smooth motion.

std_msgs/Header header

uint32 trajectory_id     # monotonic trajectory sequence ID
float32 speed            # desired max speed in m/s (0 = use param default)

# Up to 16 trajectory points.  Only the first num_points entries are valid.
uint8 num_points
float64[16] time_offsets_s   # seconds from header.stamp for each point
float64[16] latitudes
float64[16] longitudes
float32[16] speeds           # desired speed at each point (m/s)
