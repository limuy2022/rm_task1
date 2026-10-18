"""Task 1 sensor-data processing only. No drivers, serial bridge or actuators.

Requires a measured vehicle parameter file; the unmounted template is rejected.
ROS/Humble startup remains to be verified on the NUC. See the current manual.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from gimbal_tracker_bringup.deployment_config import load_vehicle_config


def _setup(context):
    cfg, params = load_vehicle_config(LaunchConfiguration('vehicle_config').perform(context))
    camera = os.path.expanduser(LaunchConfiguration('camera_config').perform(context))
    if not os.path.isfile(camera): raise ValueError('camera_config不存在')
    target = int(LaunchConfiguration('target_id').perform(context))
    if not -1 <= target < 50: raise ValueError('DICT_4X4_50的目标ID应为-1(等待)或0..49')
    submission = LaunchConfiguration('submission').perform(context).lower() == 'true'
    common = [cfg, {'config.virtual_yaml': camera, 'perception.target_digit': target,
                    'control.enabled': False, 'control.search_enabled': False}]
    nodes = [('gimbal_localization', 'base_pose_provider_node'),
             ('gimbal_localization', 'map_frame_broadcaster_node'),
             ('gimbal_localization', 'gimbal_kinematics_node'),
             ('gimbal_perception', 'aruco_detector_node'), ('gimbal_perception', 'marker_pnp_node'),
             ('gimbal_estimation', 'target_estimator_node'),
             ('grid_mapper', 'grid_mapper_node'), ('chassis_advisor', 'chassis_advisor_node')]
    result = [LogInfo(msg='仅处理已有数据；地图生产、LIO、IMU反馈和实测TF是外部前置条件；不输出电机指令')]
    parent = str(params['localization.odom_frame_id'])
    if parent != 'odom':
        xyz = params['localization.source_parent_in_output_xyz']
        rpy = params['localization.source_parent_in_output_rpy']
        args = [part for name, value in zip(('x','y','z','roll','pitch','yaw'), xyz+rpy)
                for part in ('--'+name, str(value))]
        result.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='team47_odom_source_parent', arguments=args+['--frame-id','odom','--child-frame-id',parent]))
    result.extend(Node(package=pkg, executable=exe, name=exe, output='screen', parameters=common) for pkg,exe in nodes)
    if submission:
        result.append(Node(package='gimbal_judge_io', executable='judge_io_node', name='judge_io_node',
            output='screen', parameters=common+[{'judge.fresh_run': False,
                'judge.team_id': ParameterValue(LaunchConfiguration('team_id').perform(context), value_type=str),
                'judge.output_dir': LaunchConfiguration('output_dir').perform(context)}]))
    return result


def generate_launch_description():
    share = get_package_share_directory('gimbal_tracker_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_config', description='填妥且确认的整车实测参数；不是原始模板'),
        DeclareLaunchArgument('camera_config', default_value=os.path.join(share,'config','camera_team47.yaml')),
        DeclareLaunchArgument('target_id', default_value='-1'),
        DeclareLaunchArgument('submission', default_value='false'),
        DeclareLaunchArgument('team_id', default_value='47'),
        DeclareLaunchArgument('output_dir', default_value='~/ws_championship/output'),
        OpaqueFunction(function=_setup)])
