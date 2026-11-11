#!/bin/bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$workspace_dir/src/go2_vision_test${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m go2_vision_test.review_ros "$@"
