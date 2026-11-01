"""仿真传感器参数的单位换算；不依赖ROS，不改实车参数文件。"""
import math


def use_si_acceleration(mapping):
    """把一份参数副本换成m/s²，保留原物理量程，并防止静止重力被判饱和。"""
    original_norm = float(mapping['acc_norm'])
    original_limit = float(mapping['satu_acc'])
    if not math.isfinite(original_norm) or original_norm <= 0:
        raise ValueError('原IMU加速度单位尺度必须为正有限数')
    si_limit = original_limit*9.81/original_norm
    if not math.isfinite(si_limit) or .99*si_limit <= 9.81:
        raise ValueError('IMU饱和阈值会把静止重力误判为饱和，请核对加速度单位与量程')
    mapping['satu_acc'] = si_limit
    mapping['acc_norm'] = 9.81


def simulation_lio_parameters(source, lidar_topic, imu_topic, audit_path):
    """只适配仿真坐标、单位与接口；定位算法参数沿用装车配置。"""
    result = dict(source)
    result.update(output_world_up=[0.,0.,1.],approximate_floor_from_height=False,
                  use_sim_time=True,lidar_topic=lidar_topic,imu_topic=imu_topic,
                  imu_qos_reliable=True,input_audit_path=str(audit_path),max_distance=15.)
    use_si_acceleration(result)
    return result
