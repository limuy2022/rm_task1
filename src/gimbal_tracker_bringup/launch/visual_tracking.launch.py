"""Minimal human-operated bench tracking; motion is disabled until param set.

No drivers, LIO, TF, world controller, hardware adapter, Nav2 or judge output.
with_bridge=true opens a serial port: only a person on the NUC should run it.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _setup(context):
    share = get_package_share_directory('gimbal_tracker_bringup')
    camera = LaunchConfiguration('camera_config').perform(context)
    target = int(LaunchConfiguration('target_id').perform(context))
    if not -1 <= target < 50: raise ValueError('目标ID须-1或0..49')
    common = [os.path.join(share, 'config', 'tracker_params.yaml'), {
        'config.virtual_yaml': camera, 'perception.target_digit': target,
        'perception.marker_corners_topic': '/team47_tracking/marker_corners',
        'perception.image_topic': '/image_raw', 'perception.camera_info_source': 'yaml'}]
    detector = Node(package='gimbal_perception', executable='aruco_detector_node',
        name='aruco_detector_node', namespace='team47_tracking', parameters=common, output='screen')
    pnp = Node(package='gimbal_perception', executable='marker_pnp_node', name='marker_pnp_node',
        namespace='team47_tracking', parameters=common, output='screen',
        remappings=[('/perception/marker_cam', '/team47_tracking/marker_cam')])
    controller = Node(package='gimbal_control', executable='visual_servo_node',
        name='visual_servo_node', namespace='team47_tracking', output='screen',
        parameters=[LaunchConfiguration('controller_params').perform(context),
                    {'camera_config': camera, 'motion_enabled': False}])
    nodes = [detector, pnp, controller]
    if LaunchConfiguration('with_bridge').perform(context).lower() == 'true':
        nodes.append(Node(package='gimbal_serial_42', executable='robo_bridge_node',
            name='robo_bridge', output='screen', parameters=[{
                'port': LaunchConfiguration('port').perform(context), 'baudrate': 921600,
                'subscribe_cmd_vel': True, 'publish_tf': False,
                'imu_topic': '/red_standard_robot1/gimbal',
                'cmd_vel_topic': '/team47_tracking/cmd_vel',
                'yaw_sign': 1, 'pitch_sign': 1, 'imu_axis_sign': [1, 1, 1]}]))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='tracking process exited'))])) for node in nodes]
    observers = []
    debug_csv = LaunchConfiguration('debug_csv').perform(context)
    if LaunchConfiguration('with_debug').perform(context).lower() == 'true' or debug_csv:
        observers.append(Node(package='gimbal_control', executable='tracking_debug_node',
            name='tracking_debug_node', namespace='team47_tracking', output='screen',
            parameters=[{'camera_config': camera, 'csv_path': debug_csv}]))
    return nodes + handlers + observers


def generate_launch_description():
    share = get_package_share_directory('gimbal_tracker_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('target_id', default_value='-1'),
        DeclareLaunchArgument('with_bridge', default_value='false'),
        DeclareLaunchArgument('with_debug', default_value='false'),
        DeclareLaunchArgument('debug_csv', default_value=''),
        DeclareLaunchArgument('controller_params', default_value=os.path.join(share, 'config', 'visual_tracking.yaml')),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('camera_config', default_value=os.path.join(share, 'config', 'camera_team47.yaml')),
        OpaqueFunction(function=_setup)])
