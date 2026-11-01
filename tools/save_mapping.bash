#!/usr/bin/env bash
# 独立建图结束前保存快照到records；不改固定交付地图。
set -eo pipefail
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$TRACK_WS/tools/env.bash"
export ROS_LOCALHOST_ONLY=1
exec python3 -B "$TRACK_WS/tools/save_mapping.py" --archive-dir "$TRACK_WS/records/mapping_archives" "$@"
