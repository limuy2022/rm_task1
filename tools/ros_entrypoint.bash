#!/usr/bin/env bash
# 容器提供与比赛机已有驱动相同的Livox消息依赖，不启动传感器驱动。
set -e
source /opt/ros/humble/setup.bash
source /opt/livox/install/setup.bash
exec "$@"
