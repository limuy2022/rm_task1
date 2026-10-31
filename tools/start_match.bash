#!/usr/bin/env bash
# 从本队目录启动实机全链；ROS setup脚本不支持nounset。
set -eo pipefail
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} == --help ]]; then
  echo '用法：bash tools/start_match.bash target_id:=本轮ID [vehicle_config:=绝对路径] [camera_config:=绝对路径] [motion_enabled:=true|false] [continuous:=true|false]'
  echo '首次安装见docs/教程/整体部署.md；准备/启动/停止见docs/教程/比赛启动与收尾.md。'
  exit 0
fi

source /opt/ros/humble/setup.bash
source "$TRACK_WS/.livox/env.bash"
source "$TRACK_WS/install/local_setup.bash"

# 个人装车配置优先；没有个人文件时直接使用随包安装的本轮近似配置。
VEHICLE_CONFIG="$TRACK_WS/vehicle43.yaml"
if [[ ! -f "$VEHICLE_CONFIG" ]]; then
  VEHICLE_CONFIG="$(ros2 pkg prefix --share team43_bringup)/config/vehicle_field.yaml"
fi
printf '装车配置：%s\n' "$VEHICLE_CONFIG"

export ROS_LOCALHOST_ONLY=1
mkdir -p "$TRACK_WS/records" "$TRACK_WS/config"
MATCH_RUN="$(mktemp -d "$TRACK_WS/records/match_XXXXXXXX")"
LIVOX_JSON="$TRACK_WS/config/livox_mid360.json"
if [[ ! -f $LIVOX_JSON ]]; then
  ros2 run team43_bringup configure_livox --output "$LIVOX_JSON"
fi
printf '%s\n' "本次记录目录：$MATCH_RUN" '直接启动全链，默认等待裁判目标ID；motion_enabled=true、continuous=true，不等待人工确认或旧epoch匹配。'
# 终端日志与配置放本队目录，不向正式输出写入调试文件。
export ROS_LOG_DIR="$MATCH_RUN/ros_log"
exec ros2 launch team43_bringup match.launch.py \
  vehicle_config:="$VEHICLE_CONFIG" livox_config:="$LIVOX_JSON" \
  run_dir:="$MATCH_RUN" "$@"
