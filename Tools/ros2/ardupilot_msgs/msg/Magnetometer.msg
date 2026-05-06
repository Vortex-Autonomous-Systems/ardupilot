# Magnetometer sensor message for DDS transport
# Magnetic field and calibration data

std_msgs/Header header

uint8 instance_id               # Sensor instance (0-indexed)
geometry_msgs/Vector3 field     # Magnetic field (milliGauss, body frame)
geometry_msgs/Vector3 offsets   # Current calibration offsets
float32 declination_deg         # Magnetic declination
bool healthy                    # Sensor health
bool calibrating                # Calibration in progress
bool is_primary                 # True if this is the primary compass
