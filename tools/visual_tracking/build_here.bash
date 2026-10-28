#!/usr/bin/env bash
# Only build the private workspace contained in the delivered ZIP.
set -eo pipefail
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
[[ -f "$TRACK_WS/COLCON_IGNORE" && -d "$TRACK_WS/src/team43_control" ]] || { echo '请在解压后的 43-visual-tracking 文件夹运行本脚本。'; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo '未找到 /opt/ros/humble/setup.bash；请在 NUC 的 Ubuntu 终端运行。'; exit 2; }
# 定位包依赖比赛机已有的 Livox 消息库，保留终端已加载的驱动环境。
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
command -v colcon >/dev/null || { echo 'NUC 未找到 colcon，请保留报错，不要自行安装。'; exit 2; }
ros2 pkg prefix livox_ros_driver2 >/dev/null || { echo '请先加载比赛机现有 Livox 工作区的 install/setup.bash，再重新编译本包。'; exit 2; }
/usr/bin/python3 -c 'import cv2, yaml; assert hasattr(cv2, "aruco"), "当前 OpenCV 缺少 aruco"' || { echo 'NUC 现有 Python 依赖不齐，请把上面报错发回来。'; exit 2; }
colcon --log-base "$TRACK_WS/log" build --base-paths "$TRACK_WS/src" \
  --packages-select team43_common team43_interfaces team43_perception team43_control team43_bringup team43_serial_bridge team43_judge_io team43_grid_mapper team43_localization team43_estimation team43_chassis_advisor team43_lio_backend \
  --build-base "$TRACK_WS/build" --install-base "$TRACK_WS/install" \
  --cmake-args -DBUILD_TESTING=OFF
echo "编译完成。下一步：bash \"$TRACK_WS/run_here.bash\" target_id:=0"
