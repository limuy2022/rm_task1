#!/usr/bin/env bash
# 各入口统一加载本队环境；路径不依赖调用终端的当前目录。
TEAM43_ENV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$TEAM43_ENV_ROOT/.livox/env.bash"
# 保留当前src内构建方式，同时支持部署指南中的独立安装目录。
if [[ -f "$TEAM43_ENV_ROOT/src/install/local_setup.bash" ]]; then
  source "$TEAM43_ENV_ROOT/src/install/local_setup.bash"
else
  source "$TEAM43_ENV_ROOT/install/local_setup.bash"
fi
