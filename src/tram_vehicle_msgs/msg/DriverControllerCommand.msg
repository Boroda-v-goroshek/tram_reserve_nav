# Driver controller command
# Reconstructed from the recorded rosbag wire format (verified byte-for-byte
# against /vehicle/driver_position_cmd across multiple bags and both vehicles).
std_msgs/Header header
# Driver controller position (notch):
#   0        -> neutral
#   +1..+15  -> traction (acceleration)
#   -1..-15  -> braking
int8 position
