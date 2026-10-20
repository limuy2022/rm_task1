# NUC 等价验证环境：ROS2 Humble 桌面全量镜像原样自带运行栈
# （rclpy / apt python3-opencv 4.5.4 / numpy / cv_bridge / PIL / yaml），
# 本层只补测试运行器 pytest。刻意不装 pip、不用 pip 覆盖系统库——
# 测试环境必须与运行环境一致（README：不要在 ROS/OpenCV 上叠 pip 安装）。
#
# 构建镜像：
#   docker build -t rm-task1-humble-test .
#
# 全量验证（colcon 构建 + C++ gtest + pytest + tools 套件）：
#   docker run --rm -v "$PWD":/workspace -w /workspace \
#     --user "$(id -u):$(id -g)" -e HOME=/tmp rm-task1-humble-test bash -c '
#       source /opt/ros/humble/setup.bash &&
#       colcon --log-base /tmp/clog build --build-base /tmp/cb \
#              --install-base /tmp/ci --symlink-install &&
#       source /tmp/ci/setup.bash &&
#       colcon --log-base /tmp/clog test --build-base /tmp/cb \
#              --install-base /tmp/ci --packages-select gimbal_serial_42 &&
#       colcon --log-base /tmp/clog test-result --test-result-base /tmp/cb --verbose &&
#       python3 -B -m pytest -q src --import-mode=importlib &&
#       python3 -B tools/verify_all_offline.py'
#
# 说明：
# - 仓库挂载到 /workspace（ASCII 路径，规避宿主路径含中文的问题）；
# - --user 指定宿主 uid/gid，产物属主与宿主一致，不会产生 root 属主文件；
# - colcon 的 build/install/log 全部放 /tmp，仓库内不落构建产物。

FROM osrf/ros:humble-desktop-full AS ros-base

RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pytest python3-serial \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# 可选 Gazebo / Point-LIO 验证环境：docker build --target gazebo-sim
# 源码来自仓库已有导航压缩包；编译和安装均在镜像内，不污染挂载仓库。
FROM ros-base AS gazebo-sim
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz ros-humble-pcl-ros ros-humble-pcl-conversions \
      ros-humble-ament-cmake-auto libgoogle-glog-dev libunwind-dev \
      libtbb-dev libglm-dev libomp-dev libgl1 libgl1-mesa-dri libegl1-mesa \
 && rm -rf /var/lib/apt/lists/*
COPY archives/robomaster_navigation.tar.gz /tmp/navigation.tar.gz
RUN mkdir -p /opt/lio_sim/src \
 && tar -xzf /tmp/navigation.tar.gz -C /opt/lio_sim/src --strip-components=2 \
      src/pb2025_sentry_nav/point_lio src/pb2025_sentry_nav/livox_ros_driver2 \
 && rm /tmp/navigation.tar.gz
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    cd /opt/lio_sim && colcon --log-base /tmp/lio_log build \
      --build-base /tmp/lio_build --install-base /opt/lio_sim/install \
      --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release' \
 && rm -rf /tmp/lio_build /tmp/lio_log
COPY tools/gazebo/CMakeLists.txt tools/gazebo/force_controller.cpp /opt/gazebo_controller/
RUN cmake -S /opt/gazebo_controller -B /tmp/gazebo_controller \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/lio_sim \
 && cmake --build /tmp/gazebo_controller --parallel 2 \
 && cmake --install /tmp/gazebo_controller \
 && rm -rf /tmp/gazebo_controller

# 默认目标仍为原来的轻量测试环境；CI 与本地标准命令保持一致。
FROM ros-base AS test
