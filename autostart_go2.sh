#!/bin/bash
set -eo pipefail
source /opt/venv/bin/activate
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export PYTHONUNBUFFERED=1

# Start sensors and consumers automatically; a new waypoint requests a mission.
exec ros2 launch go2_navigation waypoint_diretto.launch.py enable_motion:=true start_imu_ext:=true start_detector:=false
