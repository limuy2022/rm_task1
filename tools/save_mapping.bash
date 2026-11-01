#!/usr/bin/env bash
# 上午结束前保存并固定可供下午加载的地图；成功后才停止建图。
set -eo pipefail
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$TRACK_WS/.livox/env.bash"
source "$TRACK_WS/install/local_setup.bash"
export ROS_LOCALHOST_ONLY=1
exec python3 -B "$TRACK_WS/tools/save_mapping.py" --maps-dir "$TRACK_WS/maps" "$@"
