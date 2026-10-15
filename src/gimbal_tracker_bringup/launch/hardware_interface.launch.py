"""单一42串口桥+适配；默认只读，不启控制器/TF/硬件运动。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bridge=os.path.join(get_package_share_directory('gimbal_serial_42'),'launch','gimbal_serial.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('port',default_value='/dev/gimbal'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge),launch_arguments={
            'port':LaunchConfiguration('port'),'subscribe_cmd_vel':'true',
            'publish_tf':'false','imu_topic':'/red_standard_robot1/gimbal',
            'cmd_vel_topic':'/offboardlink/cmd_vel_gimbal'}.items()),
        Node(package='gimbal_serial_bridge',executable='hardware_adapter_node',
             name='hardware_adapter_node',output='screen',parameters=[{'control_enabled':False}]),
    ])
