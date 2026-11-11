# Custom stamped Twist: session identity prevents stale mission command replay.
# header.frame_id must equal command_frame (base_link by default).
std_msgs/Header header
string bridge_epoch
string run_id
uint64 sequence
builtin_interfaces/Duration valid_for
# Only linear.x, linear.y and angular.z supported; SI units.
geometry_msgs/Twist twist
