# Proximity sensor message for DDS transport
# Multi-sector obstacle detection

std_msgs/Header header

uint8 instance_id           # Sensor instance (0-indexed)
float32[72] distances       # Array of distances for each sector (meters)
float32 angle_offset        # Starting angle offset in radians
float32 angle_width         # Width of each sector in radians
float32 min_distance_m      # Minimum measurable distance
float32 max_distance_m      # Maximum measurable distance
uint8 num_sectors           # Number of valid sectors
uint8 status                # Sensor status

# Status constants
uint8 STATUS_NOT_CONNECTED = 0
uint8 STATUS_NO_DATA = 1
uint8 STATUS_GOOD = 2
