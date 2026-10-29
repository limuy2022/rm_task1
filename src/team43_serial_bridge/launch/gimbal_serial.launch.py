"""启动 team43_serial_bridge 串口桥。

    ros2 launch team43_serial_bridge gimbal_serial.launch.py                         # 真机（/dev/gimbal）
    ros2 launch team43_serial_bridge gimbal_serial.launch.py port:=/tmp/ttyVGIMBAL   # 虚拟云台
    RM_SIM_SERIAL=/tmp/ttyVGIMBAL ros2 launch team43_serial_bridge gimbal_serial.launch.py
    ros2 launch team43_serial_bridge gimbal_serial.launch.py subscribe_cmd_vel:=true
    ros2 launch team43_serial_bridge gimbal_serial.launch.py respawn:=true           # 设备还没插上时每秒重试

IMU 默认发布到 /red_standard_robot1/gimbal（赛题一要求，可用 imu_topic:= 覆盖）。
respawn 默认 false：串口打不开就直接报错退出（契约第 2 节，不静默重试）。
respawn:=true 时也只在“设备文件不存在”（节点退出码 3：ENOENT、ENODEV、ENXIO）才重启；
端口被占用（EBUSY、EWOULDBLOCK）或参数非法（退出码 1）一律不重试。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 只有用户在命令行上给了值，才覆盖 yaml；留空表示沿用 yaml。
_OVERRIDES = {
    'port': str,
    'imu_topic': str,
    'publish_tf': lambda s: s.lower() in ('1', 'true', 'yes'),
    'subscribe_cmd_vel': lambda s: s.lower() in ('1', 'true', 'yes'),
    'yaw_sign': int,
    'pitch_sign': int,
}


# 与 bridge_node.hpp 的 kExitDeviceAbsent 一致
_EXIT_DEVICE_ABSENT = 3
_RESPAWN_DELAY = 1.0


def _bridge_actions(params, respawn):
    node = Node(
        package='team43_serial_bridge',
        executable='robo_bridge_node',
        name='robo_bridge',
        output='screen',
        emulate_tty=True,
        parameters=params,
    )
    if not respawn:
        return [node]

    def _on_exit(event, context):
        # 只有“设备文件还不存在”才等一会儿重试；被占用和其他错误一律不重启。
        if event.returncode == _EXIT_DEVICE_ABSENT:
            return [TimerAction(period=_RESPAWN_DELAY, actions=_bridge_actions(params, respawn))]
        return []

    return [node, RegisterEventHandler(OnProcessExit(target_action=node, on_exit=_on_exit))]


def _launch_setup(context, *args, **kwargs):
    params = [LaunchConfiguration('params_file').perform(context)]
    overrides = {}
    for name, cast in _OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = cast(value)
    if overrides:
        params.append(overrides)
    respawn = LaunchConfiguration('respawn').perform(context).lower() in ('1', 'true', 'yes')
    return _bridge_actions(params, respawn)


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('team43_serial_bridge'), 'config', 'bridge.yaml')
    args = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'respawn', default_value='false',
            description='true：仅在设备文件不存在时每秒重试；被占用（EBUSY、EWOULDBLOCK）永不重试'),
    ]
    for name in _OVERRIDES:
        args.append(DeclareLaunchArgument(
            name, default_value='', description='留空沿用 yaml：' + name))
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
