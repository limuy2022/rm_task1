#!/usr/bin/env bash
# 只读录制；复用已经运行的驱动，不启动设备或运动控制。
set -eo pipefail
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TRACK_WS"
if [[ ${1:-} == --help ]]; then
  python3 -B "$TRACK_WS/tools/record.py" --help
  exit 0
fi
source /opt/ros/humble/setup.bash
source "$TRACK_WS/.livox/env.bash"
source "$TRACK_WS/install/local_setup.bash"
export ROS_LOCALHOST_ONLY=1
exec python3 -B "$TRACK_WS/tools/record.py" "$@"
