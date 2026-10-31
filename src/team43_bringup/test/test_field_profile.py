"""本轮机械初值、雷达/IMU杆臂和启动姿态参考的一致性验证。"""
from pathlib import Path
import json
import math

import numpy as np
import pytest

from team43_common.config_loader import load_ros_params
from team43_common import frames as FR
from team43_bringup.deployment_config import load_vehicle_config
from team43_bringup.lio_geometry import approximate_floor
from team43_localization.gimbal_kinematics import GimbalKinematics, SynchronizedKinematics
from team43_localization.odometry_geometry import rigid_transform, inverse_rigid, quaternion_matrix

ROOT = Path(__file__).resolve().parents[3]


def output_rotation(lio):
    # 与输出适配的几何定义独立核对：把已有重力方向转到世界+z。
    up=np.asarray(lio['output_world_up'],float);up/=np.linalg.norm(up)
    v=np.cross(up,[0.,0.,1.]);K=np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
    return np.eye(3)+K+K@K/(1+up[2])


def profile_model():
    _, p = load_vehicle_config(ROOT/'src/team43_bringup/config/vehicle_field.yaml', require_confirmation=False)
    model = GimbalKinematics({k.removeprefix('kinematics.'): v for k,v in p.items()
                             if k.startswith('kinematics.')}, require_confirmation=False)
    return p, model


def test_measured_camera_lidar_and_ground_share_one_reference():
    p, model = profile_model()
    lio = load_ros_params(ROOT/'src/team43_lio_backend/config/small_point_lio_mid360.yaml')
    T_bi = rigid_transform(p['localization.sensor_in_base_xyz'], p['localization.sensor_in_base_rpy'])
    T_il = rigid_transform(lio['extrinsic_T'], [0.,0.,0.])
    T_bl = T_bi @ T_il
    # 安装平移是银圈代理点，内部IMU偏移只计算一次，不能直接抄银圈给body。
    np.testing.assert_allclose(T_bl[:3,3], [.175,0.,-.0075], atol=1e-12)
    up = np.asarray(lio['output_world_up']); up /= np.linalg.norm(up)
    np.testing.assert_allclose(T_bi[:3,:3] @ up, [0.,0.,1.], atol=1e-12)
    np.testing.assert_allclose(model.forward(0.,0.).base_camera[:3,3], [.0625,.03,.165], atol=1e-12)
    # 已调平的LIO初始body旋转与安装旋转相消，初始底盘保持水平。
    T_wi = np.eye(4);T_wi[:3,:3]=output_rotation(lio)
    T_wb = T_wi @ inverse_rigid(T_bi)
    np.testing.assert_allclose(T_wb[2,:3], [0.,0.,1.], atol=1e-12)
    assert T_bi[0,2] > 0 and abs(T_bi[1,2]) < 1e-12  # 近似法线水平投影朝车头。
    floor, confirmed, source = approximate_floor(lio)
    assert confirmed and source == 'height_approximation'
    assert lio['lidar_height_m'] == pytest.approx(.2325)
    assert floor-T_wb[2,3] == pytest.approx(-.24)  # B为离地240mm的上表面圆心。
    np.testing.assert_allclose(p['kinematics.yaw_limits_rad'], np.deg2rad([-80,80]))
    np.testing.assert_allclose(p['kinematics.pitch_limits_rad'], np.deg2rad([-45,60]))
    assert p['kinematics.align_world_at_start'] is True
    assert p['kinematics.calibration_confirmed'] is False  # 近似不是实测精度验收。


def test_startup_reference_resolves_arbitrary_imu_world_then_tracks_motion():
    p, model = profile_model()
    sync = SynchronizedKinematics(model, bind_current_epoch=True, startup_reference=[0.,0.], max_gap=.12)
    sync.observe_epoch(123)
    R_ow = FR.rpy_to_mat([.3,-.2,.5])
    R_ob = FR.rpy_to_mat([.1,.05,.2])
    R_wg = R_ow.T @ R_ob @ model.forward(0.,0.).base_gimbal[:3,:3]
    assert sync.add_base(1., FR.compose_T(R_ob,[2.,3.,.4]), 1.)
    assert sync.add_imu(1., R_wg, 1.)
    first = sync.drain(1.)
    assert len(first) == 1
    np.testing.assert_allclose([first[0].joints.yaw, first[0].joints.pitch], [0.,0.], atol=1e-12)
    np.testing.assert_allclose(model.R_odom_world, R_ow, atol=1e-12)
    R_ob2 = FR.rpy_to_mat([.05,-.02,.25])
    R_wg2 = R_ow.T @ R_ob2 @ model.forward(.4,-.3).base_gimbal[:3,:3]
    sync.add_base(1.1, FR.compose_T(R_ob2,[2.1,3.,.4]), 1.1)
    sync.add_imu(1.1, R_wg2, 1.1)
    moved = sync.drain(1.1)
    assert len(moved) == 1
    np.testing.assert_allclose([moved[0].joints.yaw,moved[0].joints.pitch], [.4,-.3], atol=1e-12)
    assert sync.alignment_stamp == 1.
    sync.observe_epoch(456)
    assert sync.fault and not sync.enabled


def test_recorded_static_stream_bootstraps_with_approved_reference():
    p, model = profile_model()
    sync = SynchronizedKinematics(model, bind_current_epoch=True, startup_reference=[0.,0.], max_gap=.12)
    T_bi = rigid_transform(p['localization.sensor_in_base_xyz'],p['localization.sensor_in_base_rpy'])
    R = output_rotation(load_ros_params(ROOT/'src/team43_lio_backend/config/small_point_lio_mid360.yaml'))
    count = 0
    with (ROOT/'data/static_20261031_154411/measurements.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line); role = row['role']; data = row['data']; now = row['receipt_ros_s']
            if role == 'epoch': sync.observe_epoch(data['data'])
            elif role in ('lio_odom','gimbal_imu'):
                h = data['header']['stamp']; stamp = h['sec']+h['nanosec']*1e-9
                if role == 'lio_odom':
                    q=data['pose']['pose']['orientation']; pos=data['pose']['pose']['position']
                    T=np.eye(4);T[:3,:3]=R@quaternion_matrix([q[k] for k in ('x','y','z','w')])
                    T[:3,3]=R@np.array([pos[k] for k in ('x','y','z')])
                    sync.add_base(stamp,T@inverse_rigid(T_bi),now)
                else:
                    q=data['orientation']
                    sync.add_imu(stamp,quaternion_matrix([q[k] for k in ('x','y','z','w')]),now)
            count += len(sync.drain(now))
    # 验证旧采样在“首姿态视为参考”的近似下能解算，不宣称采样姿态是独立真值。
    assert sync.alignment_stamp is not None
    assert count > 100 and not sync.fault


def test_runtime_node_publishes_startup_alignment_and_marks_reset():
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import UInt64
    from team43_localization.gimbal_kinematics_node import GimbalKinematicsNode
    from team43_localization.odometry_geometry import matrix_quaternion
    args = ['--ros-args', '--params-file', str(ROOT/'src/team43_bringup/config/vehicle_field.yaml')]
    for name in ('/tf','/tf_static','/gimbal/joint_states','/task1/gimbal_reset_epoch',
                 '/gimbal/base_pose_raw','/red_standard_robot1/gimbal'):
        args += ['-r', name+':=/test/field_profile'+name]
    rclpy.init(args=args)
    node = None
    try:
        node = GimbalKinematicsNode()
        node.on_epoch(UInt64(data=999))
        stamp = node.get_clock().now().to_msg()
        base = Odometry(); base.header.stamp=stamp
        base.header.frame_id='odom';base.child_frame_id='base_link';base.pose.pose.orientation.w=1.
        node.on_base(base)
        R_ow = FR.rpy_to_mat([.3,-.2,.5])
        imu = Imu();imu.header.stamp=stamp;imu.header.frame_id='gimbal_link'
        q = matrix_quaternion(R_ow.T)
        imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w=map(float,q)
        node.on_imu(imu); node.tick()
        assert node.sent_static and node.sync.last_stamp >= 0
        assert node.get_parameter('kinematics.startup_alignment_ready').value is True
        np.testing.assert_allclose(np.asarray(node.get_parameter('kinematics.runtime_world_rotation').value).reshape(3,3),R_ow,atol=1e-12)
        node.on_epoch(UInt64(data=1000))
        assert node.sync.fault
        assert node.get_parameter('kinematics.startup_alignment_ready').value is False
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
