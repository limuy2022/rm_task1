"""`bearing_hardware.launch.py` — 实机轴承闭环组合入口（默认全关）。

组合：视觉（aruco+PnP） + 硬件串口链（C++ 串口桥+适配器） + 关节角派生（kinematics）
+ 轴承闭环（gimbal_bearing_node）。与旧 `gimbal_controller_node`（world 链）
发布同一 ``/gimbal/cmd_omega``：**不得同时启动**，也不要与 competition/sim 主链
中的 vgimbal 桥并存。

════════════════════════════════════════════════════════════════════════
一、上机前置（缺一不开环；详见实验室手册 B5 检查单）
════════════════════════════════════════════════════════════════════════
* NUC：Ubuntu 22.04 + ROS 2 Humble，本仓库已 colcon build 并 source。
* 0x01 反馈确认：串口唯一、IMU 话题持续新鲜（本队实测有效串口 /dev/ttyUSB0，
  ``robo_bridge`` 发布 ``/red_standard_robot1/gimbal``，Publisher count 应为 1）。
* 轴向核对：按协议§13 手转验证并写定 ``yaw_sign``/``pitch_sign``
  （桥启动参数；方向反了停机改这里，不得改误差符号绕过）。
* 整车参数实测：四个 kinematics 标定开关置 true、几何按实测填写；
  ``calibrated_epoch`` 每次上电会话重新对齐后更新。
* 相机-云台**旋转**外参 ≈ 0（有旋转本入口直接拒动）；
  ``perception.marker_size_m`` 用尺复核 0.10 m。

════════════════════════════════════════════════════════════════════════
二、分步启动（每步观察通过后再进下一步，现场观察人手握急停）
════════════════════════════════════════════════════════════════════════
::

    # 第1步：完全只读——四道开关全关，云台不会动
    ros2 launch team43_bringup bearing_hardware.launch.py \\
      vehicle_config:=<实测整车参数.yaml> target_id:=<官方ID> port:=/dev/ttyUSB0

    # 通过判据：/gimbal/joint_states 持续新鲜（kinematics 无 fault）；
    #           对准实体码时 /perception/marker_cam valid=true

    # 第2步：桥开始订阅指令（后面两道开关仍关，云台仍不动）
    #   重启并追加：allow_command_subscription:=true

    # 第3步：低速闭环
    #   重启并追加：control_enabled:=true bearing_enabled:=true geometry_verified:=true
    #   通过判据：对准码后云台朝码转动并收敛；方向反→立即停，查 yaw_sign/pitch_sign
    #   30秒验收：角误差 RMS<2°、无持续振荡/越界、码稳定留在图像中心区
    #   停止行为：遮挡码≥max_age → valid=false 减速停止；MCU复位 → epoch-reset，
    #             需关→开 control.bearing.enabled 重新上膛

════════════════════════════════════════════════════════════════════════
三、launch 参数（启动时指定）
════════════════════════════════════════════════════════════════════════
=========================  =============================================
vehicle_config              必填。实测整车参数；未测模板会被 load_vehicle_config 拒绝
target_id                   -1=等待（默认）；官方 ID 0..49
port                        默认 /dev/gimbal；本队机器用 /dev/ttyUSB0
camera_config               相机标定 yaml（默认 camera_team43.yaml）
allow_command_subscription  false→true：C++ 串口桥开始订阅指令（轴向核对后才可开）
control_enabled             false→true：适配器开始转发指令（低速首动前保持 false）
bearing_enabled             control.bearing.enabled；亦可运行时 ros2 param 改
geometry_verified           相机-云台旋转外参实测确认后置 true
=========================  =============================================

════════════════════════════════════════════════════════════════════════
四、运行时动态参数（ros2 param set，无需重启）
════════════════════════════════════════════════════════════════════════
全部挂在 /gimbal_bearing_node 下（适配器开关在 /hardware_adapter_node）::

    ros2 param set /gimbal_bearing_node control.bearing.enabled true

* ``control.bearing.enabled``      总开关；关→开上升沿清 epoch fault（重新上膛）
* ``control.kp_yaw / kp_pitch``    ★最重要：仿真实测 kp=6 在 0.5s 检测延迟下
                                    过冲3倍丢码死锁——首次上机 0.5 起步，稳了再升
* ``control.bearing.max_age_s``    观测最大龄；按实测"图像stamp→marker_cam到达"
                                    延迟的 2 倍设置（默认 0.15s）
* ``control.omega_max_yaw/pitch``  首动压到 0.5 rad/s（协议上限 2 不是首动建议值）
* ``control.omega_slew``           斜坡（rad/s²，默认 15，与固件一致）
* ``gimbal.yaw_limits / pitch_limits / limit_margin``   软限位（关节角域）
* ``perception.target_digit``      目标 ID 运行时切换
* ``/hardware_adapter_node control_enabled``            适配器每拍读参数，可热切换

需重启才生效：``yaw_sign``/``pitch_sign``（桥启动参数）、相机标定 yaml
（含 marker_size_m）、整车参数里的 kinematics 几何与 calibrated_epoch。

════════════════════════════════════════════════════════════════════════
五、调参顺序与更多细节
════════════════════════════════════════════════════════════════════════
只读跑通数据链 → kp=0.5/omega_max=0.5 验证转向正确 → 按 30s 闭环表现逐步升
kp 与 omega_max → 遮挡测试停止行为。每步通过判据见手册 B5 表格；
仿真证据与坑（残留进程、延迟-增益稳定性、日志修复）见
docs/验证/轴承闭环仿真验证.md。正式默认值集中在
config/tracker_params.yaml 的 control.bearing.* 段；仿真专用参数隔离在
config/vehicle_sim.yaml，不照抄到实机。
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

from team43_bringup.deployment_config import load_vehicle_config


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

    share = get_package_share_directory('team43_bringup')
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
        Node(package='team43_perception', executable='aruco_detector_node',
             name='aruco_detector_node', output='screen',
             parameters=common + [vision_overrides]),
        Node(package='team43_perception', executable='marker_pnp_node',
             name='marker_pnp_node', output='screen',
             parameters=common + [vision_overrides]),
        Node(package='team43_localization', executable='gimbal_kinematics_node',
             name='gimbal_kinematics_node', output='screen', parameters=common),
        Node(package='team43_control', executable='gimbal_bearing_node',
             name='gimbal_bearing_node', output='screen',
             parameters=common + [bearing_overrides]),
    ]
    return [LogInfo(msg='[bearing_hardware] 实机轴承闭环组合：开关默认全关，'
                        '按 D5 检查单分步打开 allow_command_subscription → '
                        'control_enabled → bearing_enabled+geometry_verified'), *nodes]


def generate_launch_description():
    share = get_package_share_directory('team43_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_config', description='实测确认的整车参数；未测模板会被拒绝'),
        DeclareLaunchArgument('camera_config',
                              default_value=os.path.join(share, 'config', 'camera_team43.yaml')),
        DeclareLaunchArgument('target_id', default_value='-1', description='官方告知的 ArUco ID'),
        DeclareLaunchArgument('port', default_value='/dev/gimbal'),
        DeclareLaunchArgument('allow_command_subscription', default_value='false',
                              description='C++ 串口桥订阅 /offboardlink/cmd_vel_gimbal；轴向核对后才可开'),
        DeclareLaunchArgument('control_enabled', default_value='false',
                              description='适配器指令转发；低速首动前保持 false'),
        DeclareLaunchArgument('bearing_enabled', default_value='false',
                              description='control.bearing.enabled；可运行时 ros2 param 改'),
        DeclareLaunchArgument('geometry_verified', default_value='false',
                              description='相机-云台旋转外参实测确认后置 true'),
        OpaqueFunction(function=_setup),
    ])
