# RangeFinder sensor message for DDS transport
# Multi-instance support with orientation and status

std_msgs/Header header

uint8 instance_id           # Sensor instance (0-indexed)
uint8 orientation           # Rotation enum (0=down, 24=forward, etc.)
float32 distance_m          # Distance in meters
float32 min_distance_m      # Minimum measurable distance
float32 max_distance_m      # Maximum measurable distance
int8 signal_quality         # Signal quality 0-100, -1 = unknown
uint8 status                # Sensor status

# Status constants
uint8 STATUS_NOT_CONNECTED = 0
uint8 STATUS_NO_DATA = 1
uint8 STATUS_OUT_OF_RANGE_LOW = 2
uint8 STATUS_OUT_OF_RANGE_HIGH = 3
uint8 STATUS_GOOD = 4
