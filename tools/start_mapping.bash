#!/usr/bin/env bash
# 上午建图入口：没有串口桥、运动控制或评分提交。
set -eo pipefail
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TRACK_WS"
if [[ ${1:-} == --help ]]; then
  echo '上午建图：bash tools/start_mapping.bash [lio_config:=绝对路径]'
  echo '结束前另开终端bash tools/save_mapping.bash，成功后Ctrl+C。下午改用start_match.bash。'
  exit 0
fi
source /opt/ros/humble/setup.bash
source "$TRACK_WS/.livox/env.bash"
source "$TRACK_WS/install/local_setup.bash"
export ROS_LOCALHOST_ONLY=1
mkdir -p "$TRACK_WS/records" "$TRACK_WS/config"
SURVEY_RUN="$(mktemp -d "$TRACK_WS/records/survey_XXXXXXXX")"
LIVOX_JSON="$TRACK_WS/config/livox_mid360.json"
if [[ ! -f $LIVOX_JSON ]]; then
  ros2 run team43_bringup configure_livox --output "$LIVOX_JSON"
fi
export ROS_LOG_DIR="$SURVEY_RUN/ros_log"
printf '上午只建图，不控制云台；本次记录：%s\n结束前另开终端执行：bash tools/save_mapping.bash\n' "$SURVEY_RUN"
exec ros2 launch team43_bringup survey.launch.py \
  livox_config:="$LIVOX_JSON" output_dir:="$SURVEY_RUN/source_map" "$@"
