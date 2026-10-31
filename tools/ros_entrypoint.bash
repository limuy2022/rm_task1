#!/usr/bin/env bash
# 容器提供本队Livox驱动及消息依赖；加载环境本身不启动硬件。
set -e
TEAM43_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
source /opt/ros/humble/setup.bash
source /opt/livox/install/setup.bash
# 实际启动驱动需要SDK；仅加载ROS消息包并不会补齐该动态库目录。
case "$(uname -m)" in
  x86_64) LIVOX_SDK_ARCH=amd64 ;;
  aarch64) LIVOX_SDK_ARCH=arm64 ;;
esac
export LD_LIBRARY_PATH="/opt/livox/src/livox_ros_driver2/Livox-SDK2/lib/$LIVOX_SDK_ARCH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# 仅可选仿真镜像包含修复收尾循环引用的 Gazebo 桥接覆盖层。
if [[ -f /opt/ros_gz/install/setup.bash ]]; then
  source /opt/ros_gz/install/setup.bash
fi
# 在所有setup之后恢复通信范围，避免环境钩子重置；与校内赛NUC默认一致。
export ROS_LOCALHOST_ONLY="$TEAM43_LOCALHOST_ONLY"
exec "$@"
