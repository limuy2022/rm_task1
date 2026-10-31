"""Only cloud->grid; uses already running Point-LIO. No sensor/motor/TF node."""
import os
from team43_common.config_loader import load_ros_params
from team43_bringup.lio_geometry import approximate_floor
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    lio_file=LaunchConfiguration('lio_params_file').perform(context)
    mapping=load_ros_params(lio_file)
    if bool(mapping.get('extrinsic_est_en',False)):
        raise ValueError('cloud mapper requires fixed LIO extrinsic_T; estimated extrinsic is not in Odometry')
    offset=[float(x) for x in mapping['extrinsic_T']]
    if len(offset)!=3: raise ValueError('Small Point-LIO extrinsic_T须含三个数值')
    floor, confirmed, source = approximate_floor(mapping)
    floor_arg = LaunchConfiguration('floor_z').perform(context)
    confirmed_arg = LaunchConfiguration('floor_confirmed').perform(context)
    if floor_arg != 'auto':
        floor = float(floor_arg)
        source = 'manual'
    if confirmed_arg != 'auto': confirmed = confirmed_arg.lower() == 'true'
    return [Node(package='team43_grid_mapper',executable='cloud_mapping_node',name='cloud_mapping_node',
                 namespace='team43',output='screen',parameters=[
                     LaunchConfiguration('params_file').perform(context),{
                         'sensor_origin_in_body':offset,'origin_model_confirmed':True,
                         'floor_z':floor, 'floor_source':source,
                         'floor_confirmed':confirmed,
                         'output_dir':LaunchConfiguration('output_dir').perform(context),
                     }])]


def generate_launch_description():
    share=get_package_share_directory('team43_grid_mapper')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','cloud_mapping.yaml')),
        DeclareLaunchArgument('lio_params_file',default_value=os.path.join(get_package_share_directory('team43_lio_backend'),'config','small_point_lio_mid360.yaml')),
        DeclareLaunchArgument('floor_z',default_value='auto'),
        DeclareLaunchArgument('floor_confirmed',default_value='auto'),
        DeclareLaunchArgument('output_dir',default_value='~/ws_championship/output/team43_mapping'),
        OpaqueFunction(function=_setup)])
