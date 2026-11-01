"""上午15分钟独立建图：只启动雷达、LIO、源栅格，不开相机控制/串口/评分。"""
import importlib.util
import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from team43_bringup.parameter_files import package_config


def _setup(context):
    share=get_package_share_directory('team43_bringup')
    value=lambda key:LaunchConfiguration(key).perform(context)
    def setup(name,args):
        path=os.path.join(share,'launch',name+'.launch.py')
        spec=importlib.util.spec_from_file_location('survey_'+name,path)
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        child=LaunchContext();child.launch_configurations.update(args)
        return mod._setup(child)
    nodes=setup('livox',{'config_file':value('livox_config')})
    nodes.append(Node(package='team43_lio_backend',executable='small_point_lio_node',
        name='team43_small_point_lio',namespace='red_standard_robot1',output='screen',
        parameters=[value('lio_config'),{'use_sim_time':False,
            'lidar_topic':'/red_standard_robot1/livox/lidar','imu_topic':'/red_standard_robot1/livox/imu'}]))
    nodes+=setup('cloud_mapping',{'lio_params_file':value('lio_config'),
        'params_file':package_config('team43_grid_mapper','cloud_mapping.yaml'),
        'floor_z':'auto','floor_confirmed':'auto','output_dir':value('output_dir')})
    return [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='上午建图进程退出'))])) for node in nodes]+nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('livox_config'),DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('lio_config',default_value=package_config('team43_lio_backend','small_point_lio_mid360.yaml')),
        OpaqueFunction(function=_setup)])
