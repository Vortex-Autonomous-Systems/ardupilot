# Rolling-horizon path command for DDS OFFBOARD mode.
# The companion sends a window of upcoming waypoints; the vehicle
# consumes them in order using AR_WPNav with SCurve look-ahead.

std_msgs/Header header

uint32 path_id          # monotonic path sequence ID
float32 speed           # desired max speed in m/s (0 = use param default)

# Up to 16 waypoints in this segment.  Only the first num_points entries
# are valid.  Each waypoint is (latitude_deg, longitude_deg).
uint8 num_points
float64[16] latitudes
float64[16] longitudes
