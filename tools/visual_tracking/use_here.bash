#!/usr/bin/env bash
# Source this file in each additional NUC terminal; no startup files are edited.
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ ! -f "$TRACK_WS/install/local_setup.bash" || ! -f /opt/ros/humble/setup.bash ]]; then
  echo '先在 NUC 执行本包的 build_here.bash，成功后再 source use_here.bash。'
  return 2 2>/dev/null || exit 2
fi
# 保留已加载的Livox依赖，再叠加本队工作区。
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/humble/setup.bash
source "$TRACK_WS/install/local_setup.bash"
export ROS_LOCALHOST_ONLY=1
TRACK_PREFIX="$(ros2 pkg prefix team43_control)" || { return 2 2>/dev/null || exit 2; }
case "$TRACK_PREFIX" in
  "$TRACK_WS/install/"*) echo "已选中本次程序：$TRACK_PREFIX" ;;
  *) echo "错误：选中了其他版本 $TRACK_PREFIX；停止，不要启动云台。"; return 2 2>/dev/null || exit 2 ;;
esac
export TRACK_WS
