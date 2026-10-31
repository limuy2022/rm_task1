"""实机比赛全链入口：先启动设备，待本次标定/地面就绪后自动接入处理链。"""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription,
                            RegisterEventHandler, EmitEvent, LogInfo)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from team43_bringup.parameter_files import package_config
from team43_bringup.deployment_config import load_vehicle_config
from team43_bringup.livox_config import load_driver_config


def _critical(node):
    return RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='比赛关键进程退出'))]))


def _ready(context):
    def value(name): return LaunchConfiguration(name).perform(context)
    snapshot = os.path.join(value('run_dir'), 'vehicle_verified.yaml')
    _, params = load_vehicle_config(snapshot)
    share = get_package_share_directory('team43_bringup')
    processing = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'processing.launch.py')),
        launch_arguments={'vehicle_config': snapshot, 'camera_config': value('camera_config'),
            'target_id': value('target_id'), 'with_perception': 'false', 'shutdown_on_exit': 'true',
            'submission': 'true', 'team_id': '43', 'output_dir': value('output_dir')}.items())
    controller = Node(package='team43_control', executable='visual_servo_node',
        name='visual_servo_node', namespace='team43_tracking', output='screen',
        parameters=[value('controller_params'), {'camera_config': value('camera_config'),
            'motion_enabled': False, 'operating_mode': 'match', 'axes': 'both',
            'corners_topic': '/perception/marker_corners',
            'joint_limits_confirmed': True, 'calibrated_epoch': params['kinematics.calibrated_epoch'],
            'joint_yaw_limits_rad': params['kinematics.yaw_limits_rad'],
            'joint_pitch_limits_rad': params['kinematics.pitch_limits_rad']}])
    return [LogInfo(msg='准备完成；运动仍关闭，请按比赛教程人工开启'), processing, _critical(controller), controller]


def _setup(context):
    def value(name): return LaunchConfiguration(name).perform(context)
    target = int(value('target_id'))
    if not 0 <= target < 50: raise ValueError('比赛入口必须提供主办方指定target_id：0..49')
    run_dir = Path(value('run_dir')).expanduser().resolve()
    if not run_dir.is_dir(): raise ValueError('run_dir须为本次新建的记录目录；请使用tools/start_match.bash')
    if (run_dir/'vehicle_verified.yaml').exists(): raise ValueError('run_dir已用，请为本次会话使用新目录')
    for name in ('camera_config', 'controller_params', 'lio_config'):
        if not os.path.isfile(value(name)): raise ValueError(name+'不存在')
    driver_path, _ = load_driver_config(value('livox_config'))
    # 直接复用各单模块入口创建的Node，所有关键进程纳入同一退出监督。
    import importlib.util
    share = get_package_share_directory('team43_bringup')
    def module(name):
        path = os.path.join(share, 'launch', name+'.launch.py')
        spec = importlib.util.spec_from_file_location('team43_match_'+name, path)
        mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        return mod
    # 通过独立LaunchContext传入参数，避免覆写父入口的同名参数。
    from launch import LaunchContext
    def setup(name, args):
        child = LaunchContext()
        child.launch_configurations.update(args)
        return module(name)._setup(child)
    hardware = setup('livox', {'config_file': str(driver_path)})
    hardware += [Node(package='team43_serial_bridge', executable='robo_bridge_node',
        name='robo_bridge', output='screen', parameters=[{'port': value('port'),
            'subscribe_cmd_vel': True, 'publish_tf': False,
            'imu_topic': '/red_standard_robot1/gimbal', 'cmd_vel_topic': '/team43_tracking/cmd_vel',
            'yaw_sign': 1, 'pitch_sign': 1, 'imu_axis_sign': [1, 1, 1]}])]
    hardware += [Node(package='team43_lio_backend', executable='small_point_lio_node',
        name='team43_small_point_lio', namespace='red_standard_robot1', output='screen',
        parameters=[value('lio_config'), {'use_sim_time': False,
            'lidar_topic': '/red_standard_robot1/livox/lidar', 'imu_topic': '/red_standard_robot1/livox/imu'}])]
    hardware += setup('cloud_mapping', {'lio_params_file': value('lio_config'),
        'params_file': package_config('team43_grid_mapper', 'cloud_mapping.yaml'),
        'floor_z': 'auto', 'floor_confirmed': 'auto', 'output_dir': str(run_dir/'source_map')})
    common = [package_config('team43_perception'), os.path.join(share, 'config', 'tracker_params.yaml'),
        {'config.virtual_yaml': value('camera_config'), 'perception.target_digit': target,
         'perception.marker_corners_topic': '/perception/marker_corners',
         'perception.image_topic': '/image_raw', 'perception.camera_info_source': 'yaml',
         'perception.marker_size_m': .075, 'perception.aruco_dict': 'DICT_4X4_50'}]
    for executable in ('aruco_detector_node', 'marker_pnp_node'):
        hardware.append(Node(package='team43_perception', executable=executable,
            name=executable, parameters=common, output='screen'))
    gate = Node(package='team43_bringup', executable='match_session_gate', output='screen',
        parameters=[{'vehicle_config': value('vehicle_config'),
                     'snapshot': str(run_dir/'vehicle_verified.yaml')}])
    def on_gate_exit(event, context):
        if context.is_shutdown: return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='比赛准备进程异常退出'))]
        return [OpaqueFunction(function=_ready)]
    return [LogInfo(msg='准备阶段：等待本次世界对齐；地面采用LIO配置指定依据，相机由赛事方启动')] + [
        _critical(node) for node in hardware] + hardware + [
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=on_gate_exit)), gate]


def generate_launch_description():
    share = get_package_share_directory('team43_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('target_id'), DeclareLaunchArgument('vehicle_config'),
        DeclareLaunchArgument('livox_config'), DeclareLaunchArgument('run_dir'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('camera_config', default_value=os.path.join(share, 'config', 'camera_team43.yaml')),
        DeclareLaunchArgument('controller_params', default_value=package_config('team43_control', 'visual_tracking.yaml')),
        DeclareLaunchArgument('lio_config', default_value=package_config('team43_lio_backend', 'small_point_lio_mid360.yaml')),
        DeclareLaunchArgument('output_dir', default_value='~/ws_championship/output'),
        OpaqueFunction(function=_setup)])
