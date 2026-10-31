"""单一 C++ 串口桥+适配；默认只读，不启控制器/TF/硬件运动。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bridge=os.path.join(get_package_share_directory('team43_serial_bridge'),'launch','gimbal_serial.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('port',default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('allow_command_subscription',default_value='false'),
        DeclareLaunchArgument('control_enabled',default_value='false',
                              description='适配器指令转发总开关；true 前必须完成 D5 检查单前置'),
        DeclareLaunchArgument('yaw_sign',default_value='1',
                              description='协议§13 轴向核对结论，透传给C++ 串口桥'),
        DeclareLaunchArgument('pitch_sign',default_value='1',
                              description='协议§13 轴向核对结论，透传给C++ 串口桥'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge),launch_arguments={
            'port':LaunchConfiguration('port'),'subscribe_cmd_vel':LaunchConfiguration('allow_command_subscription'),
            'publish_tf':'false','imu_topic':'/red_standard_robot1/gimbal',
            'cmd_vel_topic':'/offboardlink/cmd_vel_gimbal',
            'yaw_sign':LaunchConfiguration('yaw_sign'),
            'pitch_sign':LaunchConfiguration('pitch_sign')}.items()),
        Node(package='team43_gimbal_io',executable='hardware_adapter_node',
             name='hardware_adapter_node',output='screen',
             parameters=[{'control_enabled':ParameterValue(LaunchConfiguration('control_enabled'),value_type=bool)}]),
    ])
