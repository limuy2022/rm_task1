"""`bearing_hardware.launch.py` — 实机轴承闭环组合入口（默认全关）。

组合：视觉（aruco+PnP） + 硬件串口链（42桥+适配器） + 关节角派生（kinematics）
+ 轴承闭环（gimbal_bearing_node）。与旧 `gimbal_controller_node`（world 链）
发布同一 ``/gimbal/cmd_omega``：**不得同时启动**，也不要与 competition/sim 主链
中的 vgimbal 桥并存。

四道开关全部默认关闭，按实验室手册 D5 检查单**分步**打开：

1. ``ros2 launch gimbal_tracker_bringup bearing_hardware.launch.py \\
      vehicle_config:=<实测整车参数> target_id:=<官方ID>``（全部只读）
2. 确认 0x01 反馈与关节角新鲜（B3 入口 / ``/gimbal/joint_states``）
3. 轴向核对（协议§13 手转 + ``yaw_sign``/``pitch_sign``）后：
   ``allow_command_subscription:=true``
4. 低速闭环： ``control_enabled:=true bearing_enabled:=true geometry_verified:=true``

``vehicle_config`` 必须是**实测确认**的整车参数（四个 kinematics 标定开关 +
``calibrated_epoch`` 已按当前上电会话填写）；未测模板会被 ``load_vehicle_config``
直接拒绝。首次 kp 建议 0.5~1.0 量级低速试起，按实测检测延迟整定
（仿真证据见 docs/当前任务/轴承闭环仿真验证.md：kp=6 在 0.5s 检测延迟下发散）。
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, \
    LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from gimbal_tracker_bringup.deployment_config import load_vehicle_config


def _setup(context):
    cfg, params = load_vehicle_config(LaunchConfiguration('vehicle_config').perform(context))
    camera = os.path.expanduser(LaunchConfiguration('camera_config').perform(context))
    if not os.path.isfile(camera):
        raise ValueError('camera_config 不存在: %s' % camera)
    target = int(LaunchConfiguration('target_id').perform(context))
    if not -1 <= target < 50:
        raise ValueError('DICT_4X4_50 的目标 ID 应为 -1(等待) 或 0..49')
    if params.get('control.enabled', False):
        raise ValueError('整车参数含 control.enabled=true；本入口只驱动轴承闭环，'
                         '不得与旧 world 控制器同时使能')

    share = get_package_share_directory('gimbal_tracker_bringup')
    tracker = os.path.join(share, 'config', 'tracker_params.yaml')
    vision_overrides = {
        'config.virtual_yaml': camera,
        'perception.target_digit': target,
    }
    bearing_overrides = {
        'control.bearing.enabled':
            ParameterValue(LaunchConfiguration('bearing_enabled'), value_type=bool),
        'control.bearing.geometry_verified':
            ParameterValue(LaunchConfiguration('geometry_verified'), value_type=bool),
    }
    common = [tracker, cfg]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'hardware_interface.launch.py')),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'allow_command_subscription': LaunchConfiguration('allow_command_subscription'),
            'control_enabled': LaunchConfiguration('control_enabled'),
        }.items())

    nodes = [
        hardware,
        Node(package='gimbal_perception', executable='aruco_detector_node',
             name='aruco_detector_node', output='screen',
             parameters=common + [vision_overrides]),
        Node(package='gimbal_perception', executable='marker_pnp_node',
             name='marker_pnp_node', output='screen',
             parameters=common + [vision_overrides]),
        Node(package='gimbal_localization', executable='gimbal_kinematics_node',
             name='gimbal_kinematics_node', output='screen', parameters=common),
        Node(package='gimbal_control', executable='gimbal_bearing_node',
             name='gimbal_bearing_node', output='screen',
             parameters=common + [bearing_overrides]),
    ]
    return [LogInfo(msg='[bearing_hardware] 实机轴承闭环组合：开关默认全关，'
                        '按 D5 检查单分步打开 allow_command_subscription → '
                        'control_enabled → bearing_enabled+geometry_verified'), *nodes]


def generate_launch_description():
    share = get_package_share_directory('gimbal_tracker_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_config', description='实测确认的整车参数；未测模板会被拒绝'),
        DeclareLaunchArgument('camera_config',
                              default_value=os.path.join(share, 'config', 'camera_team47.yaml')),
        DeclareLaunchArgument('target_id', default_value='-1', description='官方告知的 ArUco ID'),
        DeclareLaunchArgument('port', default_value='/dev/gimbal'),
        DeclareLaunchArgument('allow_command_subscription', default_value='false',
                              description='42桥订阅 /offboardlink/cmd_vel_gimbal；轴向核对后才可开'),
        DeclareLaunchArgument('control_enabled', default_value='false',
                              description='适配器指令转发；低速首动前保持 false'),
        DeclareLaunchArgument('bearing_enabled', default_value='false',
                              description='control.bearing.enabled；可运行时 ros2 param 改'),
        DeclareLaunchArgument('geometry_verified', default_value='false',
                              description='相机-云台旋转外参实测确认后置 true'),
        OpaqueFunction(function=_setup),
    ])
