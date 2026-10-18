#!/usr/bin/env bash
set -eo pipefail
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$TRACK_WS/use_here.bash"
exec ros2 launch gimbal_tracker_bringup visual_tracking.launch.py "$@"
