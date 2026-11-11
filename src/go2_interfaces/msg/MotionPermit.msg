# Supervisor asserts: localisation ready AND validated mission explicitly RUNNING.
# Header stamp: system/ROS wall time, clocks synchronised. No use_sim_time.
std_msgs/Header header
string bridge_epoch
# Unique per explicit mission start/resume, never an empty string.
string run_id
uint64 sequence
bool enabled
builtin_interfaces/Duration valid_for
