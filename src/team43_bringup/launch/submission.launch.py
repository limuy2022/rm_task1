"""仅启动赛题一评分输出；不启动传感器、串口或运动控制。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('team_id', default_value='43'),
        DeclareLaunchArgument('output_dir', default_value='~/ws_championship/output'),
        DeclareLaunchArgument('tick_topic', default_value='/judge/tick'),
        DeclareLaunchArgument('replay_topic', default_value='/judge/replay_start'),
        Node(package='team43_judge_io', executable='judge_io_node', output='screen',
             parameters=[{'judge.team_id': ParameterValue(LaunchConfiguration('team_id'), value_type=str),
                          'judge.output_dir': LaunchConfiguration('output_dir'),
                          'judge.fresh_run': False,
                          'judge.enforce_v_floor': False}],
             remappings=[('/judge/tick', LaunchConfiguration('tick_topic')),
                         ('/judge/replay_start', LaunchConfiguration('replay_topic'))]),
    ])
