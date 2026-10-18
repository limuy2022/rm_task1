"""`bearing_bench.launch.py` — 台架试转入口：静止底盘 + 真串口 + 真相机。

适用场景：**车身固定不动**（轮子锁死/压重、上电时大致放平），只验证
"图像 → 检测 → PnP → 轴承闭环 → 串口 → 云台转向码" 这条链本身。
底盘位姿用 mock 静止源——对完全静止的台架，恒等位姿就是真值，不是近似。
移动/比赛场景必须换 ``bearing_hardware.launch.py`` + 实测整车参数。

📋 可打印的完整操作手册（含命令、判据与故障处置表）：
docs/教程/云台台架试转步骤.md

════════════════════════════════════════════════════════════════════════
一、与实机入口的差异（台架假设的代价，接受才使用）
════════════════════════════════════════════════════════════════════════
* 不经过 ``load_vehicle_config`` 实车校验；几何全零假设。
* yaw 关节角带未知常数偏差（IMU 上电朝向随机）→ PC 软限位不可信；
  依赖固件编码器限位（协议§10.3）+ 低速 + 人工断电兜底。
* pitch 关节角是重力基准的，仍然正确；瞄准误差来自图像，不受影响。

════════════════════════════════════════════════════════════════════════
二、运行前 3 分钟准备（绕不过的运行条件）
════════════════════════════════════════════════════════════════════════
1. 读本会上电的 epoch（每次上电都会变，不能编）::

       ros2 topic echo /task1/gimbal_reset_epoch --once \\
         --qos-reliability reliable --qos-durability transient_local

2. 确认 0x01 反馈持续新鲜（imu_topic 唯一发布者、stamp 连续）。
3. 轴向手转核对（协议§13）：正 yaw_omega ↔ gz 同向、正 pitch_omega ↔ gy 同向；
   结论写 42 桥的 ``yaw_sign``/``pitch_sign``（本入口参数）。

════════════════════════════════════════════════════════════════════════
三、分步启动（四道开关默认全关，逐步打开；全程低速+手握电源）
════════════════════════════════════════════════════════════════════════
::

    # 第1步：只读（epoch 换成刚读到的数值；轴向已核对则带 yaw_sign/pitch_sign）
    ros2 launch gimbal_tracker_bringup bearing_bench.launch.py \\
      epoch:=<刚读到的epoch> target_id:=<打印码的ID> port:=/dev/ttyUSB0

    # 通过判据：/gimbal/joint_states 持续新鲜；对准打印码时
    #           /perception/marker_cam valid=true；joint pitch 随手动俯仰变化合理

    # 第2步：桥订阅指令（后两道开关仍关，云台不动）
    #   重启追加：allow_command_subscription:=true

    # 第3步：低速闭环（kp/omega_max 已在 vehicle_bench 预设低速值）
    #   重启追加：control_enabled:=true bearing_enabled:=true geometry_verified:=true
    #   判据：对准码后云台朝码平滑转动、码回到图像中心；方向反→停，查 yaw_sign

════════════════════════════════════════════════════════════════════════
四、参数
════════════════════════════════════════════════════════════════════════
``epoch``（必填，本会上电值） / ``target_id`` / ``port``（默认 /dev/ttyUSB0） /
``camera_config`` / ``yaw_sign``、``pitch_sign``（默认 +1，轴向核对结论） /
``allow_command_subscription`` / ``control_enabled`` / ``bearing_enabled`` /
``geometry_verified``（四道开关，默认全 false）。
运行时动态调参（kp、omega_max、max_age 等）速查见
``gimbal_control/gimbal_bearing_node.py`` 文件头；kp 必须按实测检测延迟整定
（仿真证据：kp=6 在 0.5s 检测延迟下过冲丢码）。
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


def _setup(context):
    share = get_package_share_directory('gimbal_tracker_bringup')
    epoch = int(LaunchConfiguration('epoch').perform(context))
    if epoch < 0:
        raise ValueError('epoch 必填：先 ros2 topic echo /task1/gimbal_reset_epoch '
                         '--once --qos-reliability reliable --qos-durability transient_local')
    target = int(LaunchConfiguration('target_id').perform(context))
    if not 0 <= target < 50:
        raise ValueError('台架试转需对准实体码：target_id 应为 0..49')
    camera = os.path.expanduser(LaunchConfiguration('camera_config').perform(context))
    if not os.path.isfile(camera):
        raise ValueError('camera_config 不存在: %s' % camera)

    tracker = os.path.join(share, 'config', 'tracker_params.yaml')
    bench = os.path.join(share, 'config', 'vehicle_bench.yaml')
    common = [tracker, bench, {'kinematics.calibrated_epoch': epoch,
                               'perception.target_digit': target}]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'hardware_interface.launch.py')),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'allow_command_subscription': LaunchConfiguration('allow_command_subscription'),
            'control_enabled': LaunchConfiguration('control_enabled'),
            'yaw_sign': LaunchConfiguration('yaw_sign'),
            'pitch_sign': LaunchConfiguration('pitch_sign'),
        }.items())

    bearing_overrides = {
        'control.bearing.enabled':
            ParameterValue(LaunchConfiguration('bearing_enabled'), value_type=bool),
        'control.bearing.geometry_verified':
            ParameterValue(LaunchConfiguration('geometry_verified'), value_type=bool),
    }

    nodes = [
        hardware,
        Node(package='gimbal_localization', executable='base_pose_provider_node',
             name='base_pose_provider_node', output='screen', parameters=common),
        Node(package='gimbal_perception', executable='aruco_detector_node',
             name='aruco_detector_node', output='screen', parameters=common),
        Node(package='gimbal_perception', executable='marker_pnp_node',
             name='marker_pnp_node', output='screen', parameters=common),
        Node(package='gimbal_localization', executable='gimbal_kinematics_node',
             name='gimbal_kinematics_node', output='screen', parameters=common),
        Node(package='gimbal_control', executable='gimbal_bearing_node',
             name='gimbal_bearing_node', output='screen',
             parameters=common + [bearing_overrides]),
    ]
    return [LogInfo(msg='[bearing_bench] 台架试转（静止底盘+真串口）：epoch=%d '
                        'target=%d；台架假设见 vehicle_bench.yaml 头注；四道开关'
                        '默认全关，分步打开' % (epoch, target)), *nodes]


def generate_launch_description():
    share = get_package_share_directory('gimbal_tracker_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('epoch', default_value='-1',
                              description='本会上电的 /task1/gimbal_reset_epoch 值（必填）'),
        DeclareLaunchArgument('target_id', default_value='0'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('camera_config',
                              default_value=os.path.join(share, 'config', 'camera_team47.yaml')),
        DeclareLaunchArgument('yaw_sign', default_value='1'),
        DeclareLaunchArgument('pitch_sign', default_value='1'),
        DeclareLaunchArgument('allow_command_subscription', default_value='false'),
        DeclareLaunchArgument('control_enabled', default_value='false'),
        DeclareLaunchArgument('bearing_enabled', default_value='false'),
        DeclareLaunchArgument('geometry_verified', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
