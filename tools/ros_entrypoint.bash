#!/usr/bin/env bash
# 容器提供与比赛机已有驱动相同的Livox消息依赖，不启动传感器驱动。
set -e
source /opt/ros/humble/setup.bash
source /opt/livox/install/setup.bash
# 仅可选仿真镜像包含修复收尾循环引用的 Gazebo 桥接覆盖层。
if [[ -f /opt/ros_gz/install/setup.bash ]]; then
  source /opt/ros_gz/install/setup.bash
fi
exec "$@"
