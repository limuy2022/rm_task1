"""本队硬件雷达入口：仅启动Livox，不启动定位、云台或底盘。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from team43_bringup.livox_config import load_driver_config


def _setup(context):
    path, _ = load_driver_config(LaunchConfiguration('config_file').perform(context))
    return [Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', namespace='red_standard_robot1', output='screen',
        parameters=[{
            'user_config_path': str(path),
            'xfer_format': 0,
            'multi_topic': 0, 'data_src': 0, 'output_data_type': 0,
            'publish_freq': 10.0, 'frame_id': 'livox_frame', 'use_sim_time': False,
        }],
        remappings=[
            ('livox/lidar', '/red_standard_robot1/livox/lidar'),
            ('livox/imu', '/red_standard_robot1/livox/imu'),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', description='本队生成的MID360网络JSON绝对路径'),
        OpaqueFunction(function=_setup),
    ])
