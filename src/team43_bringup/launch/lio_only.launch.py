"""只启动 Small Point-LIO 项目适配层；复用已有雷达/IMU，不启动硬件或电机。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('team43_lio_backend')
    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/red_standard_robot1/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/red_standard_robot1/livox/imu'),
        DeclareLaunchArgument('namespace', default_value='red_standard_robot1'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'small_point_lio_mid360.yaml')),
        Node(package='team43_lio_backend', executable='small_point_lio_node', name='team43_small_point_lio',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'lidar_topic': ParameterValue(LaunchConfiguration('lidar_topic'), value_type=str),
                 'imu_topic': ParameterValue(LaunchConfiguration('imu_topic'), value_type=str),
             }]),
    ])
