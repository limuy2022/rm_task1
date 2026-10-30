"""只启动图像检测和PnP；目标ID可启动后通过参数设置，无运动输出。"""
import os
from team43_bringup.parameter_files import package_config
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share = get_package_share_directory('team43_bringup')
    params = [os.path.join(share, 'config', 'tracker_params.yaml'), {
        'config.virtual_yaml': LaunchConfiguration('camera_config'),
        'perception.target_digit': ParameterValue(LaunchConfiguration('target_id'), value_type=int),
        'perception.image_topic': '/image_raw',
        'perception.marker_corners_topic': '/perception/marker_corners',
        'perception.camera_info_source': 'yaml',
        'perception.marker_size_m': 0.10,
    }]
    return LaunchDescription([
        DeclareLaunchArgument('target_id', default_value='-1'),
        DeclareLaunchArgument('camera_config', default_value=os.path.join(share,'config','camera_team43.yaml')),
        Node(package='team43_perception', executable='aruco_detector_node', name='aruco_detector_node', parameters=[package_config('team43_perception'), *params], output='screen'),
        Node(package='team43_perception', executable='marker_pnp_node', name='marker_pnp_node', parameters=[package_config('team43_perception'), *params], output='screen'),
    ])
