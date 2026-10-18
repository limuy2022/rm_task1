"""`bearing_sim.launch.py` — 轴承闭环全感知链仿真（审计 P0「控制ROS接入」验证入口）。

链路::

    virtual_mcu(pty 0x01/0x30) ← vgimbal_bridge ← gimbal_bearing_node
                                   ↑ epoch/IMU        ↑ /gimbal/cmd_omega
    virtual_vision(合成ArUco图像) → /image_raw → aruco_detector → marker_pnp
                                   → /perception/marker_cam ↗
    base_pose_provider(mock) + gimbal_kinematics → /gimbal/joint_states ↗
    e2e_evaluator（GT 角误差 RMS/max，评估完自动收尾）

与 `competition/sim` 主链的区别：

* **不含** estimator/judge/地图——轴承闭环不依赖 map/LIO；正式比赛仍走主线入口；
* 虚拟相机图像 remap 到 ``/image_raw``、内参加来自 ``/virtual_gimbal/camera_info``
  （``camera_info_source: topic``），与 src 感知管道约定一致；
* ``gimbal_kinematics_node`` 补进链路（关节角来源；主链阶梯里没有它）；
* vgimbal 桥 ``serial.publish_tf: false``——``world→gimbal_link`` 让位给
  kinematics 的 ``odom→base_link→gimbal_link`` 链，避免 gimbal_link 双父节点。

启动（Humble）::

    ros2 launch gimbal_tracker_bringup bearing_sim.launch.py target_id:=0
"""

from __future__ import annotations

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, \
    RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def _share_dir(package: str) -> str:
    return get_package_share_directory(package)


def _as_bool(text) -> bool:
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context) -> list:
    params_file = context.launch_configurations['params_file']
    vehicle = context.launch_configurations['vehicle_config']
    camera_config = context.launch_configurations['camera_config']
    target_id = int(context.launch_configurations['target_id'])
    port = context.launch_configurations['port']
    step = float(context.launch_configurations['startup_step_s'])
    run_duration = float(context.launch_configurations['run_duration_s'])
    eval_enabled = _as_bool(context.launch_configurations['eval'])
    eval_duration = float(context.launch_configurations['eval_duration'])
    eval_output = context.launch_configurations['eval_output']
    series_output = context.launch_configurations['eval_series']
    target_yaw = float(context.launch_configurations['target_yaw'])
    target_pitch = float(context.launch_configurations['target_pitch'])
    target_distance = float(context.launch_configurations['target_distance'])

    # src 节点共用：参数文件 + 仿真整车配置 + 目标 ID 覆盖
    src_params = [params_file, vehicle, {'perception.target_digit': target_id}]

    mcu = Node(package='virtual_gimbal', executable='virtual_mcu', name='virtual_mcu',
               output='screen', parameters=[{'port': port, 'imu_rate_hz': 200.0,
                                             'imu_ready_delay': 1.0,
                                             'gt_tf_rate_hz': 50.0, 'status_rate_hz': 10.0}])
    base_pose = Node(package='gimbal_localization', executable='base_pose_provider_node',
                     name='base_pose_provider_node', output='screen', parameters=src_params)
    bridge = Node(package='gimbal_serial_bridge', executable='vgimbal_bridge_node',
                  name='vgimbal_bridge_node', output='screen', parameters=src_params)
    vision = Node(package='virtual_gimbal', executable='virtual_vision', name='virtual_vision',
                  output='screen', parameters=[camera_config, {
                      'vision_mode': 'corners', 'synthetic_image': True,
                      'trajectory': 'static',
                      # static 轨迹的目标方位由 yaw_offset/pitch_offset 决定
                      # （yaw_amp/pitch_amp 只对 sine/circle/scan 生效）。
                      'yaw_offset': target_yaw, 'pitch_offset': target_pitch,
                      'target_distance': target_distance,
                      'marker_size': 0.10, 'marker_id': target_id,
                      'rate_hz': 10.0, 'gt_rate_hz': 30.0}])
    detector = Node(package='gimbal_perception', executable='aruco_detector_node',
                    name='aruco_detector_node', output='screen', parameters=src_params,
                    remappings=[('/image_raw', '/virtual_gimbal/image_raw')])
    pnp = Node(package='gimbal_perception', executable='marker_pnp_node',
               name='marker_pnp_node', output='screen', parameters=src_params)
    kinematics = Node(package='gimbal_localization', executable='gimbal_kinematics_node',
                      name='gimbal_kinematics_node', output='screen', parameters=src_params)
    bearing = Node(package='gimbal_control', executable='gimbal_bearing_node',
                   name='gimbal_bearing_node', output='screen', parameters=src_params)

    ladder: List[Node] = [mcu, base_pose, bridge, vision, detector, pnp, kinematics, bearing]
    actions: list = [LogInfo(msg='[bearing_sim] 轴承闭环仿真：目标 ID=%d，静态目标 '
                                'yaw=%.2f pitch=%.2f d=%.1fm（%s 后自动收尾）'
                                % (target_id, target_yaw, target_pitch, target_distance,
                                   ('评估完成' if eval_enabled else f'{run_duration:.0f}s')))]
    for i, node in enumerate(ladder):
        actions.append(node if i == 0 else TimerAction(period=i * step, actions=[node]))

    if eval_enabled:
        evaluator = Node(
            package='virtual_gimbal', executable='e2e_evaluator', name='e2e_evaluator',
            output='screen', parameters=[{
                'duration': eval_duration, 'rms_max_deg': 2.0, 'max_deg': 5.0,
                # 暂态窗口：初始捕获（云台从 18.5° 偏置转向目标，实测 ~2.4s）
                # 不计入稳态 max/RMS 指标；手册 D5 口径也是"收敛后 RMS"。
                'convergence_time': 4.0, 'require_swing': False,
                'min_samples': 50, 'sample_rate_hz': 50.0,
                'output_file': eval_output, 'series_file': series_output}])
        actions.append(TimerAction(period=len(ladder) * step, actions=[evaluator]))
        # 评估器退出（exit_on_done）→ 整体收尾
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=evaluator, on_exit=[EmitEvent(event=Shutdown(reason='e2e done'))])))
    else:
        actions.append(TimerAction(period=run_duration, actions=[
            LogInfo(msg='[bearing_sim] run_duration 到时 → Shutdown'),
            EmitEvent(event=Shutdown(reason='bearing_sim duration')),
        ]))
    return actions


def generate_launch_description():
    share = _share_dir('gimbal_tracker_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config',
                            'tracker_params.yaml'), description='§6.13 全表参数'),
        DeclareLaunchArgument('vehicle_config', default_value=os.path.join(share, 'config',
                            'vehicle_sim.yaml'), description='仿真整车配置（标定开关=真值模型）'),
        DeclareLaunchArgument('camera_config', default_value=os.path.join(share, 'config',
                            'camera_sim.yaml'), description='虚拟相机内参；PnP 从 CameraInfo 获取同组内参'),
        DeclareLaunchArgument('target_id', default_value='0', description='要瞄准的 ArUco ID'),
        DeclareLaunchArgument('port', default_value='/tmp/ttyVGIMBAL', description='虚拟 MCU pty'),
        DeclareLaunchArgument('startup_step_s', default_value='0.3', description='阶梯启动步长'),
        DeclareLaunchArgument('target_yaw', default_value='0.3',
                               description='静态目标方位角 rad（逆时针为正）；'
                                           '初始必须在 camera_config 指定相机的半视场内，'
                                           '否则目标不可见、观测恒空'),
        DeclareLaunchArgument('target_pitch', default_value='0.12',
                               description='静态目标俯仰角 rad（低头为正）'),
        DeclareLaunchArgument('target_distance', default_value='1.5',
                               description='目标距离 m（640 宽下 2.5m 时码仅 ~22px，1.5m≈37px 更稳）'),
        DeclareLaunchArgument('eval', default_value='true', description='是否运行 e2e 评估器'),
        DeclareLaunchArgument('eval_duration', default_value='30.0',
                               description='云台首次使能后的评估时长 s'),
        DeclareLaunchArgument('eval_output', default_value='',
                               description='评估 JSON 输出路径（空=只打印）'),
        DeclareLaunchArgument('eval_series', default_value='',
                               description='逐样本角误差 CSV 路径（空=不写）'),
        DeclareLaunchArgument('run_duration_s', default_value='60.0',
                               description='eval:=false 时的自动收尾时长 s'),
        OpaqueFunction(function=_setup),
    ])
