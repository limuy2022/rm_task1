"""Only the supplied Point-LIO executable; reuses existing sensor publishers.

No lidar/camera driver, chassis/gimbal commands or Nav2. Input CustomMsg and IMU
units must be checked first. Point-LIO must already be installed in the overlay.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('gimbal_tracker_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic',default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic',default_value='/livox/imu'),
        DeclareLaunchArgument('namespace',default_value='red_standard_robot1'),
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','point_lio_mid360.yaml')),
        Node(package='point_lio',executable='pointlio_mapping',name='team47_point_lio',
             namespace=LaunchConfiguration('namespace'),output='screen',
             parameters=[LaunchConfiguration('params_file'),{
                 'common.lid_topic':ParameterValue(LaunchConfiguration('lidar_topic'),value_type=str),
                 'common.imu_topic':ParameterValue(LaunchConfiguration('imu_topic'),value_type=str),
                 'prior_pcd.enable':False,'pcd_save.pcd_save_en':False}],
             remappings=[('/tf','tf'),('/tf_static','tf_static')])])
