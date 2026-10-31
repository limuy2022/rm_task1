"""从同一份LIO配置推导用户授权的高度近似地面，不把近似冒充点选实测。"""
import math


def approximate_floor(params):
    if params.get('approximate_floor_from_height') is not True:
        return 0., False, 'manual'
    up = [float(v) for v in params.get('output_world_up', [0., 0., 1.])]
    offset = [float(v) for v in params['extrinsic_T']]
    height = float(params['lidar_height_m'])
    if (len(up) != 3 or len(offset) != 3 or not all(math.isfinite(v) for v in up+offset)
            or not math.isfinite(height) or height <= 0):
        raise ValueError('高度近似需要有限三维重力方向/光心偏移及正高度')
    norm = math.sqrt(sum(v*v for v in up))
    if norm < 1e-6: raise ValueError('output_world_up不能为零')
    # R把up转到+z，因此(R*T).z == dot(up/|up|,T)。初始世界原点在IMU body。
    return sum(v*t/norm for v,t in zip(up,offset))-height, True, 'height_approximation'
