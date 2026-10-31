#!/usr/bin/env bash
# 裁判通知后切换共享检测器/PnP目标，无需重启比赛程序。
set -eo pipefail
if [[ $# != 1 || ! $1 =~ ^[0-9]+$ || ${#1} -gt 2 ]]; then
  echo '用法：bash tools/set_target.bash 目标ID（0..49）' >&2
  exit 2
fi
TARGET_ID=$((10#$1))
if (( TARGET_ID > 49 )); then
  echo '目标ID必须在0..49之间' >&2
  exit 2
fi
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$TRACK_WS/.livox/env.bash"
source "$TRACK_WS/install/local_setup.bash"
export ROS_LOCALHOST_ONLY=1
# 单一ROS参数客户端完成暂停、切换和读回，避免重复CLI图发现漏节点。
exec python3 -B "$TRACK_WS/tools/set_target.py" "$TARGET_ID"
