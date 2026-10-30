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
#              --install-base /tmp/ci --packages-select team43_serial_bridge team43_lio_backend &&
#       colcon --log-base /tmp/clog test-result --test-result-base /tmp/cb --verbose &&
#       python3 -B -m pytest -q src --import-mode=importlib &&
#       python3 -B tools/verify_all_offline.py --skip-pytest'
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

# 测试容器提供比赛机已有的Livox依赖；算法源码直接位于src普通ROS包中。
FROM ros-base AS livox-runtime
COPY archives/livox_ros_driver2.tar.gz /tmp/livox_ros_driver2.tar.gz
RUN echo 'b438683b99951c2a962df0919106476baf00f3281a50e5691203dc510dd24d23  /tmp/livox_ros_driver2.tar.gz' | sha256sum -c - \
 && mkdir -p /opt/livox/src \
 && tar -xzf /tmp/livox_ros_driver2.tar.gz -C /opt/livox/src \
 && rm /tmp/livox_ros_driver2.tar.gz \
 && /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    colcon --log-base /tmp/livox_log build --base-paths /opt/livox/src \
      --build-base /tmp/livox_build --install-base /opt/livox/install \
      --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
 && rm -rf /tmp/livox_build /tmp/livox_log
COPY tools/ros_entrypoint.bash /team43_ros_entrypoint.bash
ENTRYPOINT ["bash", "/team43_ros_entrypoint.bash"]

FROM ros-base AS gazebo-deps
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz ros-humble-pcl-ros ros-humble-pcl-conversions \
      ros-humble-ament-cmake-auto libgoogle-glog-dev libunwind-dev \
      libtbb-dev libglm-dev libomp-dev libgl1 libgl1-mesa-dri libegl1-mesa \
 && rm -rf /var/lib/apt/lists/*
FROM gazebo-deps AS gazebo-sim
# Humble apt 0.244.25 尚未包含上游 #956 的桥接节点循环引用修复；
# 固定上游提交，并回移 #977 的回调引用与显式 shutdown 修复。
COPY archives/ros_gz-humble.tar.gz /tmp/ros_gz-humble.tar.gz
COPY archives/ros_gz-shutdown.patch /tmp/ros_gz-shutdown.patch
RUN echo 'bf88be8c6aac9de0abdf4fc7745a1dbbcf3fcb1b948ad656999cfe61f8fc56b6  /tmp/ros_gz-humble.tar.gz' | sha256sum -c - \
 && mkdir -p /opt/ros_gz/src \
 && tar -xzf /tmp/ros_gz-humble.tar.gz -C /opt/ros_gz/src --strip-components=1 \
 && git -C /opt/ros_gz/src apply /tmp/ros_gz-shutdown.patch \
 && rm /tmp/ros_gz-humble.tar.gz /tmp/ros_gz-shutdown.patch \
 && /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    MAKEFLAGS="-j2" colcon --log-base /tmp/ros_gz_log build --base-paths /opt/ros_gz/src/ros_gz_bridge \
      --build-base /tmp/ros_gz_build --install-base /opt/ros_gz/install \
      --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
 && rm -rf /tmp/ros_gz_build /tmp/ros_gz_log /opt/ros_gz/src
COPY --from=livox-runtime /opt/livox /opt/livox
COPY tools/ros_entrypoint.bash /team43_ros_entrypoint.bash
ENTRYPOINT ["bash", "/team43_ros_entrypoint.bash"]
# 该路径仅存放Gazebo运动插件；定位包本身不依赖它。
COPY src/team43_gazebo/plugins/ /opt/gazebo_controller/
RUN cmake -S /opt/gazebo_controller -B /tmp/gazebo_controller \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/team43_gazebo \
 && cmake --build /tmp/gazebo_controller --parallel 2 \
 && cmake --install /tmp/gazebo_controller \
 && rm -rf /tmp/gazebo_controller
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/team43_gazebo/lib

FROM livox-runtime AS test
