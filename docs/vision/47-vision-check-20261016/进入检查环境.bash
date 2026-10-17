# 使用 source 加载；只改变当前终端的环境，不改系统文件或其它进程。
if [ -z "${BASH_VERSION:-}" ]; then
    printf '请在NUC的bash终端中 source 本文件。\n'
    return 1
fi
export T47_SRC="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export T47_RUN="$HOME/ws_championship/team47_check_20261016"
if [ ! -r /opt/ros/humble/setup.bash ]; then
    printf '未找到 /opt/ros/humble/setup.bash。请确认这是NUC Ubuntu终端。\n'
    return 1
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
if [ -r "$T47_RUN/install/local_setup.bash" ]; then
    source "$T47_RUN/install/local_setup.bash"
fi
mkdir -p "$T47_RUN/records"
: > "$T47_RUN/COLCON_IGNORE"
printf '队伍源码：%s\n队伍构建与记录：%s\nROS域：%s（保留原设置）\n' "$T47_SRC" "$T47_RUN" "${ROS_DOMAIN_ID:-0}"
printf '本终端只使用系统Humble和本队专用安装目录。\n'
