#!/usr/bin/env bash
set -eo pipefail
exec ros2 launch team43_bringup visual_tracking.launch.py "$@"
