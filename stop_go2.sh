#!/bin/bash
# Stop only this navigation stack, allowing ROS launch to reap its children.
set -euo pipefail
launch_pattern='^(/usr/bin/python3|/opt/venv/bin/python3) /opt/ros/jazzy/bin/ros2 launch go2_navigation waypoint_diretto[.]launch[.]py( |$)'
node_pattern='^(/usr/bin/python3|/opt/venv/bin/python3) /ws/install/(go2_sensors/lib/go2_sensors/(gps_node|imu_mag_node)|go2_navigation/lib/go2_navigation/navigation_node|go2_bridge/lib/go2_bridge/bridge|go2_animal_detector/lib/go2_animal_detector/animal_detector_node|go2_mission_manager/lib/go2_mission_manager/mission_manager_node)( |$)'
wait_gone() {
    local pattern="$1" rounds="$2"
    for ((i=0; i<rounds; i++)); do
        if ! pgrep -f "$pattern" >/dev/null; then return 0; fi
        sleep 0.2
    done
    ! pgrep -f "$pattern" >/dev/null
}
# SIGTERM on ROS Jazzy launch can orphan children; use SIGINT instead.
pkill -INT -f "$launch_pattern" || true
if ! wait_gone "$launch_pattern" 100; then
    printf '%s\n' 'Navigation launcher did not stop; refusing a duplicate start.' >&2
    exit 1
fi
# Also clear nodes orphaned by an earlier unclean exit, before a new launch.
pkill -INT -f "$node_pattern" || true
if ! wait_gone "$node_pattern" 50; then
    printf '%s\n' 'Navigation children still running; refusing a duplicate start.' >&2
    exit 1
fi
