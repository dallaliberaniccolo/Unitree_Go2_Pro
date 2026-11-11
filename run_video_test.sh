#!/bin/bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [ "$#" -lt 1 ]; then
    echo "Uso: bash $0 /percorso/video.mp4 [fps:=1.5] [loop:=true]"
    exit 2
fi
video_path="$(realpath -e -- "$1")"
shift
if [ ! -f "$video_path" ]; then
    echo "Il percorso deve indicare un file video locale: $video_path" >&2
    exit 2
fi
if [ -f /opt/venv/bin/activate ]; then
    source /opt/venv/bin/activate
fi
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif ! command -v ros2 >/dev/null 2>&1; then
    echo "ROS 2 non disponibile. Eseguire nell'ambiente ROS Jazzy dopo il build; vedere README_video_test.md." >&2
    exit 2
fi
if [ ! -f "$workspace_dir/install/setup.bash" ]; then
    echo "Workspace non compilato: manca $workspace_dir/install/setup.bash. Vedere README_video_test.md." >&2
    exit 2
fi
source "$workspace_dir/install/setup.bash"
exec ros2 launch go2_vision_test video_test.launch.py \
    "video_path:=$video_path" \
    "record_directory:=$workspace_dir/log/wolf_samples" \
    "model_path:=$workspace_dir/models/wolf/wolf_detector.pt" "$@"
