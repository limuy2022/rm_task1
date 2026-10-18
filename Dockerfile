# NUC 等价验证环境：ROS2 Humble 桌面全量镜像原样自带运行栈
# （rclpy / apt python3-opencv 4.5.4 / numpy / cv_bridge / PIL / yaml），
# 本层只补测试运行器 pytest。刻意不装 pip、不用 pip 覆盖系统库——
# 测试环境必须与运行环境一致（README：不要在 ROS/OpenCV 上叠 pip 安装）。
#
# 构建镜像：
#   docker build -t rm-task1-humble-test .
#
# 全量验证（colcon 构建 + pytest + tools 套件）：
#   docker run --rm -v "$PWD":/workspace -w /workspace \
#     --user "$(id -u):$(id -g)" -e HOME=/tmp rm-task1-humble-test bash -c '
#       source /opt/ros/humble/setup.bash &&
#       colcon --log-base /tmp/clog build --build-base /tmp/cb \
#              --install-base /tmp/ci --symlink-install &&
#       source /tmp/ci/setup.bash &&
#       python3 -B -m pytest -q src --import-mode=importlib &&
#       python3 -B tools/verify_all_offline.py'
#
# 说明：
# - 仓库挂载到 /workspace（ASCII 路径，规避宿主路径含中文的问题）；
# - --user 指定宿主 uid/gid，产物属主与宿主一致，不会产生 root 属主文件；
# - colcon 的 build/install/log 全部放 /tmp，仓库内不落构建产物。

FROM osrf/ros:humble-desktop-full

RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pytest python3-serial \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
