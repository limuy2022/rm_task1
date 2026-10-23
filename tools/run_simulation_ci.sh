#!/usr/bin/env bash
# CI 与本地共用的容器内仿真入口；不安装依赖、不修改正式参数。
set -eo pipefail

scenario="${1:-}"
case "$scenario" in
  navigation|gazebo_lio|gazebo_vision|submission) ;;
  *) echo '用法：bash tools/run_simulation_ci.sh navigation|gazebo_lio|gazebo_vision|submission' >&2; exit 2 ;;
esac

cd /workspace
output="build/offline/ci_simulation/$scenario"
# 防止重跑覆盖前一次验收证据；本地重跑前应移走已有目录。
mkdir -p "$(dirname "$output")"
mkdir "$output"
exec > >(tee "$output/console.log") 2>&1

source /opt/ros/humble/setup.bash
if [[ "$scenario" == gazebo_* ]]; then
  source /opt/lio_sim/install/setup.bash
fi
colcon --log-base /tmp/clog build --build-base /tmp/cb \
  --install-base /tmp/ci --symlink-install --parallel-workers 2
source /tmp/ci/setup.bash

if [[ "$scenario" == submission ]]; then
  python3 -B tools/verify_submission_dds.py --count 1800 --output-dir "$output/result"
elif [[ "$scenario" == navigation ]]; then
  python3 -B tools/simulate_navigation.py --scenario all \
    --output-dir "$output/result"
else
  # 降低请求倍率，为无显卡 runner 的软件渲染留出时间；不放宽误差门槛。
  # 视觉默认使用 features 场景和 3cm 观测下限，沿用已验证的基线。
  python3 -B "tools/simulate_${scenario}.py" \
    --output-dir "$output/result" --imu-mode input --scan-lines 16 \
    --real-time-factor 0.25 --timeout 900
fi
