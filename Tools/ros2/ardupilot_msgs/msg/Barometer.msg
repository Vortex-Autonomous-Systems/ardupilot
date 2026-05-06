# Barometer sensor message for DDS transport
# Pressure, temperature and altitude data

std_msgs/Header header

uint8 instance_id           # Sensor instance (0-indexed)
float32 pressure_pa         # Pressure in Pascals
float32 temperature_c       # Temperature in Celsius
float32 altitude_m          # Altitude relative to calibration
float32 altitude_amsl_m     # Altitude above mean sea level
float32 climb_rate_ms       # Vertical climb rate m/s (primary baro only)
bool healthy                # Sensor health
bool is_primary             # True if this is the primary barometer
