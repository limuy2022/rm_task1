"""单一42串口桥+适配；默认只读，不启控制器/TF/硬件运动。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bridge=os.path.join(get_package_share_directory('gimbal_serial_42'),'launch','gimbal_serial.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('port',default_value='/dev/gimbal'),
        DeclareLaunchArgument('allow_command_subscription',default_value='false'),
        DeclareLaunchArgument('control_enabled',default_value='false',
                              description='适配器指令转发总开关；true 前必须完成 D5 检查单前置'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge),launch_arguments={
            'port':LaunchConfiguration('port'),'subscribe_cmd_vel':LaunchConfiguration('allow_command_subscription'),
            'publish_tf':'false','imu_topic':'/red_standard_robot1/gimbal',
            'cmd_vel_topic':'/offboardlink/cmd_vel_gimbal'}.items()),
        Node(package='gimbal_serial_bridge',executable='hardware_adapter_node',
             name='hardware_adapter_node',output='screen',
             parameters=[{'control_enabled':ParameterValue(LaunchConfiguration('control_enabled'),value_type=bool)}]),
    ])
