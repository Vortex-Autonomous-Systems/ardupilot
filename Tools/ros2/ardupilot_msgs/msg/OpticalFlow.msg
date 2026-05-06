# OpticalFlow sensor message for DDS transport
# Flow rate and body rate data

std_msgs/Header header

uint8 instance_id                   # Sensor instance (0-indexed)
geometry_msgs/Vector3 flow_rate     # Optical flow rate rad/s (body frame)
geometry_msgs/Vector3 body_rate     # IMU-corrected body rate rad/s
uint8 quality                       # Surface quality 0-255
bool healthy                        # Sensor health status
