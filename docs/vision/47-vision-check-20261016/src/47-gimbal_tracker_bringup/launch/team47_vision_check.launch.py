"""公用NUC专用视觉检查：只读相机，业务输出位于/team47_check。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('gimbal_tracker_bringup')
    common = {
        'config.virtual_yaml': os.path.join(share, 'config', 'camera_team47.yaml'),
        'perception.target_digit': ParameterValue(LaunchConfiguration('target_id'), value_type=int),
        'perception.image_topic': '/image_raw',
        'perception.marker_corners_topic': '/team47_check/marker_corners',
        'perception.camera_info_topic': '/team47_check/camera_info_unused',
        'perception.camera_info_source': 'yaml',
        'perception.marker_size_m': 0.10,
    }
    params = [os.path.join(share, 'config', 'tracker_params.yaml'), common]
    return LaunchDescription([
        DeclareLaunchArgument('target_id', default_value='-1'),
        Node(package='gimbal_perception', executable='aruco_detector_node',
             name='aruco_detector_node', namespace='team47_check',
             parameters=params, output='screen'),
        Node(package='gimbal_perception', executable='marker_pnp_node',
             name='marker_pnp_node', namespace='team47_check',
             parameters=params, output='screen',
             remappings=[('/perception/marker_cam', '/team47_check/marker_cam')]),
    ])
