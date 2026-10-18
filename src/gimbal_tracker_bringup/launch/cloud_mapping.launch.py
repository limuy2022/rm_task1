"""Only cloud->grid; uses already running Point-LIO. No sensor/motor/TF node."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context):
    lio_file=LaunchConfiguration('lio_params_file').perform(context)
    with open(lio_file,encoding='utf-8') as f:
        cfg=yaml.safe_load(f)['/**']['ros__parameters']
    mapping=cfg['mapping']
    if bool(mapping.get('extrinsic_est_en',cfg.get('extrinsic_est_en',False))):
        raise ValueError('cloud mapper requires fixed LIO extrinsic_T; estimated extrinsic is not in Odometry')
    offset=[float(x) for x in mapping['extrinsic_T']]
    if len(offset)!=3: raise ValueError('Point-LIO mapping.extrinsic_T needs three values')
    return [Node(package='grid_mapper',executable='cloud_mapping_node',name='cloud_mapping_node',
                 namespace='team47',output='screen',parameters=[
                     LaunchConfiguration('params_file'),{
                         'sensor_origin_in_body':offset,'origin_model_confirmed':True,
                         'floor_z':ParameterValue(LaunchConfiguration('floor_z'),value_type=float),
                         'floor_confirmed':ParameterValue(LaunchConfiguration('floor_confirmed'),value_type=bool),
                         'output_dir':ParameterValue(LaunchConfiguration('output_dir'),value_type=str),
                     }])]


def generate_launch_description():
    share=get_package_share_directory('gimbal_tracker_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','cloud_mapping.yaml')),
        DeclareLaunchArgument('lio_params_file',default_value=os.path.join(share,'config','point_lio_mid360.yaml')),
        DeclareLaunchArgument('floor_z',default_value='0.0'),
        DeclareLaunchArgument('floor_confirmed',default_value='false'),
        DeclareLaunchArgument('output_dir',default_value='~/ws_championship/output/team47_mapping'),
        OpaqueFunction(function=_setup)])
