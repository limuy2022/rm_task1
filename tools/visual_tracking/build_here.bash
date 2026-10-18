#!/usr/bin/env bash
# Only build the private workspace contained in the delivered ZIP.
set -eo pipefail
TRACK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
[[ -f "$TRACK_WS/COLCON_IGNORE" && -d "$TRACK_WS/src/47-gimbal_control" ]] || { echo '请在解压后的 47-visual-tracking-20261018-r3 文件夹运行本脚本。'; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo '未找到 /opt/ros/humble/setup.bash；请在 NUC 的 Ubuntu 终端运行。'; exit 2; }
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
command -v colcon >/dev/null || { echo 'NUC 未找到 colcon，请保留报错，不要自行安装。'; exit 2; }
/usr/bin/python3 -c 'import cv2, yaml; assert hasattr(cv2, "aruco"), "当前 OpenCV 缺少 aruco"' || { echo 'NUC 现有 Python 依赖不齐，请把上面报错发回来。'; exit 2; }
colcon --log-base "$TRACK_WS/log" build --base-paths "$TRACK_WS/src" \
  --packages-select gimbal_common gimbal_interfaces gimbal_perception gimbal_control gimbal_tracker_bringup gimbal_serial_42 gimbal_judge_io grid_mapper gimbal_localization gimbal_estimation chassis_advisor \
  --build-base "$TRACK_WS/build" --install-base "$TRACK_WS/install" \
  --cmake-args -DBUILD_TESTING=OFF -DWITH_HIK_CAMERA=OFF
echo "编译完成。下一步：bash \"$TRACK_WS/run_here.bash\" target_id:=0"
