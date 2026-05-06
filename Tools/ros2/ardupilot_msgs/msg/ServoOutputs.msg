# ServoOutputs message for DDS transport
# PWM and normalized servo output values

std_msgs/Header header

uint8 num_channels          # Number of active channels
uint16[16] pwm              # PWM values for each channel (typically 1000-2000)
float32[16] output          # Normalized output values (-1 to 1)
uint16[16] function         # SRV_Channel function assigned to each channel
