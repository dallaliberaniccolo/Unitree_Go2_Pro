#!/bin/bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [ "$#" -lt 1 ]; then
    echo "Uso: bash $0 /percorso/video-o-cartella [--output /percorso/report] [--fps 1]"
    exit 2
fi
if [ -f /opt/venv/bin/activate ]; then
    source /opt/venv/bin/activate
fi
export PYTHONPATH="$workspace_dir/src/go2_animal_detector:$workspace_dir/src/go2_vision_test${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m go2_vision_test.analyze_video \
    --model "$workspace_dir/models/wolf/wolf_detector.pt" \
    --output "${GO2_VIDEO_OUTPUT:-$workspace_dir/log/video_analysis}" "$@"
