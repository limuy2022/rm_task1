"""Check explicit measured configuration before the data-processing launch."""
from pathlib import Path
import yaml
from gimbal_localization.gimbal_kinematics import GimbalKinematics
from gimbal_localization.odometry_geometry import rigid_transform


def load_vehicle_config(path):
    path = Path(path).expanduser().resolve()
    with path.open(encoding='utf-8') as stream: data = yaml.safe_load(stream)
    try: params = data['/**']['ros__parameters']
    except (TypeError, KeyError) as exc: raise ValueError('需要/**: ros__parameters:参数文件') from exc
    if not isinstance(params, dict): raise ValueError('参数必须为字典')
    for key in ('kinematics.calibration_confirmed', 'kinematics.joint_zero_known',
                'kinematics.mounted_on_base_confirmed', 'localization.sensor_extrinsics_confirmed',
                'localization.parent_transform_confirmed', 'estimation.geometry_confirmed'):
        if params.get(key) is not True: raise ValueError(f'{key}未确认；禁止把模板当作实测整车配置')
    GimbalKinematics({k.removeprefix('kinematics.'): v for k,v in params.items() if k.startswith('kinematics.')})
    for prefix in ('sensor_in_base', 'source_parent_in_output'):
        rigid_transform(params['localization.'+prefix+'_xyz'], params['localization.'+prefix+'_rpy'])
    if not isinstance(params.get('kinematics.calibrated_epoch'), int) or params['kinematics.calibrated_epoch'] < 0:
        raise ValueError('必须填写本次IMU世界对齐所对应的calibrated_epoch')
    if params.get('localization.base_pose_source') != 'odom': raise ValueError('实车不能使用mock底盘')
    if params.get('localization.origin_mode') != 'judge_replay_start': raise ValueError('实车原点必须来自replay_start')
    if params.get('grid.map_source') != 'topic' or not params.get('grid.source_frame'):
        raise ValueError('必须使用自己的真实地图话题并注明source_frame')
    # The current node interfaces use these names, so reject partial renaming.
    for key, expected in {'localization.output_frame_id':'odom', 'localization.base_frame_id':'base_link',
                          'kinematics.odom_frame':'odom', 'kinematics.base_frame':'base_link',
                          'grid.output_frame':'map'}.items():
        if params.get(key, expected) != expected: raise ValueError(f'{key}当前要求{expected}')
    if params.get('control.enabled', False): raise ValueError('processing入口不接受运动使能')
    return str(path), params
