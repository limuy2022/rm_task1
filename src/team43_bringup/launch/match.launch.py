"""下午比赛：LIO独立评分链；默认平行墙示意图，不等待旧地图重定位。"""
import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from team43_bringup.parameter_files import package_config
from team43_bringup.deployment_config import load_vehicle_config
from team43_bringup.livox_config import load_driver_config
from team43_bringup.lio_geometry import approximate_floor
from team43_common.config_loader import load_ros_params


def _critical(node):
    return RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='比赛关键进程退出'))]))


# 与 team43_serial_bridge/include/bridge_node.hpp 的 kExitDeviceAbsent 保持一致：
# 仅"设备文件还不存在"用该退出码（可等待插上后重试）；被占用等其它失败必须整链退出。
_EXIT_DEVICE_ABSENT = 3
_BRIDGE_RESPAWN_DELAY_S = 1.0


def _respawn_on_device_absent(node, make_node):
    """robo_bridge 退出监督：设备缺席→延迟重试（其余链路照常），其余退出→全链 Shutdown。"""
    def on_exit(event, context):
        if getattr(event, 'returncode', None) == _EXIT_DEVICE_ABSENT:
            again = make_node()
            return [LogInfo(msg=f'云台串口设备尚不存在，{_BRIDGE_RESPAWN_DELAY_S:.0f} 秒后重试 '
                                'robo_bridge（其余链路不受影响）'),
                    RegisterEventHandler(OnProcessExit(target_action=again, on_exit=on_exit)),
                    TimerAction(period=_BRIDGE_RESPAWN_DELAY_S, actions=[again])]
        return [EmitEvent(event=Shutdown(reason='比赛关键进程退出'))]
    return RegisterEventHandler(OnProcessExit(target_action=node, on_exit=on_exit))


def _ready(context):
    def value(name): return LaunchConfiguration(name).perform(context)
    motion = ParameterValue(LaunchConfiguration('motion_enabled', default='true'), value_type=bool).evaluate(context)
    continuous = ParameterValue(LaunchConfiguration('continuous', default='true'), value_type=bool).evaluate(context)
    snapshot = os.path.join(value('run_dir'), 'vehicle_runtime.yaml')
    _, params = load_vehicle_config(snapshot, require_confirmation=False)
    share = get_package_share_directory('team43_bringup')
    processing = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'processing.launch.py')),
        launch_arguments={'vehicle_config': snapshot, 'camera_config': value('camera_config'),
            'target_id': value('target_id'), 'with_perception': 'false', 'shutdown_on_exit': 'true',
            'submission': 'true', 'require_confirmation': 'false', 'team_id': '43', 'output_dir': value('output_dir')}.items())
    controller = Node(package='team43_control', executable='visual_servo_node',
        name='visual_servo_node', namespace='team43_tracking', output='screen',
        parameters=[value('controller_params'), {'camera_config': value('camera_config'),
            'motion_enabled': motion, 'continuous': continuous, 'operating_mode': 'match', 'axes': 'both',
            'corners_topic': '/perception/marker_corners',
            'joint_limits_confirmed': True, 'bind_current_epoch': True,
            'calibrated_epoch': int(params.get('kinematics.calibrated_epoch', -1)),
            'joint_yaw_limits_rad': params['kinematics.yaw_limits_rad'],
            'joint_pitch_limits_rad': params['kinematics.pitch_limits_rad']}])
    return [LogInfo(msg=f'比赛处理链启动；motion_enabled={str(motion).lower()}，continuous={str(continuous).lower()}'), processing, _critical(controller), controller]


def _setup(context):
    def value(name): return LaunchConfiguration(name).perform(context)
    target = int(value('target_id'))
    if not -1 <= target < 50: raise ValueError('target_id须为-1（等待裁判通知）或0..49')
    run_dir = Path(value('run_dir')).expanduser().resolve()
    if not run_dir.is_dir(): raise ValueError('run_dir须为本次新建的记录目录；请使用tools/start_match.bash')
    if (run_dir/'vehicle_runtime.yaml').exists(): raise ValueError('run_dir已用，请为本次会话使用新目录')
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
    port = value('port')
    def robo_bridge():
        return Node(package='team43_serial_bridge', executable='robo_bridge_node',
            name='robo_bridge', output='screen', parameters=[{'port': port,
                'subscribe_cmd_vel': True, 'publish_tf': False,
                'imu_topic': '/red_standard_robot1/gimbal', 'cmd_vel_topic': '/team43_tracking/cmd_vel',
                'yaw_sign': 1, 'pitch_sign': 1, 'imu_axis_sign': [1, 1, 1]}])
    bridge = robo_bridge()
    hardware += [bridge]
    hardware += [Node(package='team43_lio_backend', executable='small_point_lio_node',
        name='team43_small_point_lio', namespace='red_standard_robot1', output='screen',
        parameters=[value('lio_config'), {'use_sim_time': False,
            'lidar_topic': '/red_standard_robot1/livox/lidar', 'imu_topic': '/red_standard_robot1/livox/imu'}])]
    mode = LaunchConfiguration('map_mode', default='corridor').perform(context)
    optional_map = None
    if mode == 'live':
        hardware += setup('cloud_mapping', {'lio_params_file': value('lio_config'),
            'params_file': package_config('team43_grid_mapper', 'cloud_mapping.yaml'),
            'floor_z': 'auto', 'floor_confirmed': 'auto', 'output_dir': str(run_dir/'source_map')})
    elif mode == 'corridor':
        floor, confirmed, _ = approximate_floor(load_ros_params(value('lio_config')))
        if confirmed:
            optional_map = Node(package='team43_grid_mapper',executable='corridor_map_node',
                name='corridor_map_node',output='screen',parameters=[
                    package_config('team43_grid_mapper','corridor_model.yaml'),
                    {'floor_z':floor,'run_dir':str(run_dir)}])
            hardware.append(optional_map)
        else:
            hardware.append(LogInfo(msg='地面近似不可用，跳过可选走廊示意图；评分链继续运行'))
    elif mode == 'none':
        pass
    elif mode == 'saved':
        from team43_grid_mapper.saved_map import SavedMap
        if not value('map_yaml'):raise ValueError('下午比赛需要上午地图map_yaml；请先运行start_mapping.bash和save_mapping.bash')
        path = Path(value('map_yaml')).expanduser().resolve()
        SavedMap(path)  # 开硬件前验证文件、原点和有效地图；不等待人工确认开关。
        _, vehicle = load_vehicle_config(value('vehicle_config'), require_confirmation=False)
        floor, confirmed, _ = approximate_floor(load_ros_params(value('lio_config')))
        if not confirmed:raise ValueError('跨会话重定位需要同一下午LIO的有效地面配置')
        hardware.append(Node(package='team43_grid_mapper',executable='saved_map_node',
            name='saved_map_node',output='screen',parameters=[{'map_yaml':str(path),
                'floor_z':floor, 'run_dir':str(run_dir),
                'sensor_in_base_xyz':vehicle['localization.sensor_in_base_xyz'],
                'sensor_in_base_rpy':vehicle['localization.sensor_in_base_rpy']}]))
    else:raise ValueError('map_mode必须是corridor、none、saved或live')
    common = [package_config('team43_perception'), os.path.join(share, 'config', 'tracker_params.yaml'),
        {'config.virtual_yaml': value('camera_config'), 'perception.target_digit': target,
         'perception.marker_corners_topic': '/perception/marker_corners',
         'perception.image_topic': '/image_raw', 'perception.camera_info_source': 'yaml',
         'perception.marker_size_m': .075, 'perception.aruco_dict': 'DICT_4X4_50'}]
    for executable in ('aruco_detector_node', 'marker_pnp_node'):
        hardware.append(Node(package='team43_perception', executable=executable,
            name=executable, parameters=common, output='screen'))
    # 用户已经完成安装测量：直接使用数值，不等待人工确认开关、旧epoch或源图。
    _, params = load_vehicle_config(value('vehicle_config'), require_confirmation=False)
    params.update({'kinematics.require_confirmation': False,
                   'kinematics.bind_current_epoch': True,
                   'localization.sensor_extrinsics_confirmed': True,
                   'localization.parent_transform_confirmed': True,
                   'estimation.geometry_confirmed': True})
    if mode in ('corridor','none'):
        # 建议值不接底盘执行器；不因缺图、旧图多解或门状态变化停住CSV建议输出。
        params['chassis.dir_mode'] = 'direct'
        params['chassis.enable_avoidance'] = False
        params['chassis.map_timeout_s'] = 0.0
        floor,confirmed,_ = approximate_floor(load_ros_params(value('lio_config')))
        params.update({'chassis.obstacle_guard_enabled':True,
                       'chassis.obstacle_floor_z':floor,
                       'chassis.obstacle_floor_valid':confirmed})
    elif mode == 'saved':
        # 上午地图是静态证据；重发不能伪造新的采样stamp，也不能按实时图5秒过期。
        # 只关闭地图年龄超时，目标/底盘/观测的新鲜度保护保持原值。
        params['chassis.map_timeout_s'] = 0.0
    snapshot = run_dir/'vehicle_runtime.yaml'
    with snapshot.open('x', encoding='utf-8') as stream:
        stream.write('# 比赛运行参数；不作为实测或验收通过证据。\n')
        yaml.safe_dump({'/**': {'ros__parameters': params}}, stream, allow_unicode=True)
    # robo_bridge 单独监督：设备缺席按 bridge_respawn 延迟重试，不把全链拖死。
    respawn_bridge = LaunchConfiguration(
        'bridge_respawn', default='true').perform(context).lower() in ('1', 'true', 'yes')
    bridge_watch = (_respawn_on_device_absent(bridge, robo_bridge) if respawn_bridge
                    else _critical(bridge))
    return [LogInfo(msg=f'启动处理与控制链；map_mode={mode}；target_id=-1时等待裁判通知；corridor/none模式不等待上午地图重定位')] + [
        _critical(node) for node in hardware if isinstance(node,Node) and node is not bridge and node is not optional_map
    ] + [bridge_watch] + hardware + _ready(context)



def generate_launch_description():
    share = get_package_share_directory('team43_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('target_id', default_value='-1'), DeclareLaunchArgument('vehicle_config'),
        DeclareLaunchArgument('motion_enabled', default_value='true'),
        DeclareLaunchArgument('continuous', default_value='true'),
        DeclareLaunchArgument('map_mode', default_value='corridor'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('livox_config'), DeclareLaunchArgument('run_dir'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('bridge_respawn', default_value='true'),
        DeclareLaunchArgument('camera_config', default_value=os.path.join(share, 'config', 'camera_team43.yaml')),
        DeclareLaunchArgument('controller_params', default_value=package_config('team43_control', 'visual_tracking.yaml')),
        DeclareLaunchArgument('lio_config', default_value=package_config('team43_lio_backend', 'small_point_lio_mid360.yaml')),
        DeclareLaunchArgument('output_dir', default_value='~/ws_championship/output'),
        OpaqueFunction(function=_setup)])
