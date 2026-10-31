#!/usr/bin/env python3
"""仅在隔离容器验证比赛机安装包：发布合成雷达/IMU，不连接硬件。"""
import argparse
import json
import math
import numpy as np
import os
from pathlib import Path
import signal
import subprocess
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--field-alignment', action='store_true',
                        help='额外验证实机非单位输出旋转同时作用到位姿和点云；仍使用合成输入')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('仅允许在ROS_LOCALHOST_ONLY=1及独立ROS_DOMAIN_ID的测试容器运行')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('team43_lio_backend'))
    params = yaml.safe_load((share/'config/small_point_lio_mid360.yaml').read_text())
    config = params['/**']['ros__parameters']
    if not args.field_alignment: config['output_world_up'] = [0., 0., 1.]
    config.update(approximate_floor_from_height=False, acc_norm=9.81, satu_acc=29.43, extrinsic_T=[.1, -.02, .04])
    path = output/'test_params.yaml'
    path.write_text(yaml.safe_dump(params))
    rclpy.init()
    node = Node('team43_lio_install_test')
    poses, clouds = [], []
    node.create_subscription(Odometry, '/lio_install_test/aft_mapped_to_init', poses.append, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/lio_install_test/cloud_registered', clouds.append, qos_profile_sensor_data)
    lidar = node.create_publisher(PointCloud2, '/lio_install_test/lidar', qos_profile_sensor_data)
    imu = node.create_publisher(Imu, '/lio_install_test/imu', qos_profile_sensor_data)
    points = []
    for a in range(-10, 11):
        for b in range(-6, 7):
            for xyz in ((2., a*.15, b*.15), (a*.15, 2., b*.15), (a*.15, b*.15, -1.)):
                points.append(xyz)
    log = (output/'launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'team43_lio_backend', 'lio.launch.py',
                               'namespace:=lio_install_test', 'lidar_topic:=/lio_install_test/lidar',
                               'imu_topic:=/lio_install_test/imu', 'params_file:='+str(path)],
                              cwd=output, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    report = {'hardware_accessed': False, 'package_share': str(share), 'lidar_format': 'pointcloud2'}
    start = time.monotonic()
    next_imu = next_lidar = start
    try:
        while time.monotonic()-start < 15.:
            if process.poll() is not None:
                raise RuntimeError('定位安装包提前退出，见launch.log')
            now = time.monotonic()
            if now >= next_imu:
                msg = Imu()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.linear_acceleration.z = 9.81
                imu.publish(msg)
                next_imu = now+.005
            if now >= next_lidar:
                stamp = node.get_clock().now()
                # 官方 packed PointXYZRTLT，timestamp 为绝对纳秒；非零逐点时差。
                msg = PointCloud2()
                msg.header.stamp = stamp.to_msg()
                msg.height = 1
                msg.width = len(points)
                msg.point_step = 26
                msg.row_step = msg.width * msg.point_step
                msg.fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
                              for name, offset, datatype in (
                                  ('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7),
                                  ('tag', 16, 2), ('line', 17, 2), ('timestamp', 18, 8))]
                msg.data = b''.join(struct.pack('<ffffBBd', *p, 100., 0x10, 0,
                                               float(stamp.nanoseconds + i * 10000))
                                    for i, p in enumerate(points))
                lidar.publish(msg)
                next_lidar = now+.1
            rclpy.spin_once(node, timeout_sec=.001)
            if now-start > 5. and len(poses) >= 20 and len(clouds) >= 10:
                break
        stamps = [p.header.stamp.sec+p.header.stamp.nanosec*1e-9 for p in poses]
        checks = {
            'odometry_received': len(poses) >= 20,
            'cloud_received': len(clouds) >= 10,
            'body_frame': bool(poses) and all(p.header.frame_id == 'lio_world' and p.child_frame_id == 'body' for p in poses),
            'cloud_world_frame': bool(clouds) and all(c.header.frame_id == 'lio_world' and c.width > 0 for c in clouds),
            'finite_static_body_pose': bool(poses) and all(math.isfinite(v) and abs(v) < .05 for p in poses
                for v in (p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z)),
            'timestamps_increase': len(stamps) > 1 and all(b > a for a, b in zip(stamps, stamps[1:])),
        }
        if args.field_alignment and poses and clouds:
            from team43_common.frames import quat_to_mat
            from team43_grid_mapper.cloud_grid import pointcloud2_xyz
            q = poses[-1].pose.pose.orientation
            rotation = quat_to_mat([q.w, q.x, q.y, q.z])
            up = np.asarray(config['output_world_up']); up /= np.linalg.norm(up)
            checks['nonidentity_pose_aligns_configured_up'] = bool(np.linalg.norm(rotation @ up-[0.,0.,1.]) < .03)
            # 合成房间的原始三面墙；将输出云逆转回原世界应恢复几何。
            xyz = pointcloud2_xyz(clouds[-1]) @ rotation - np.asarray(config['extrinsic_T'])
            errors = np.minimum(np.minimum(abs(xyz[:,0]-2.),abs(xyz[:,1]-2.)),abs(xyz[:,2]+1.))
            checks['cloud_and_pose_use_same_rotation'] = bool(np.mean(errors < .1) > .98)
            report['inverse_rotated_cloud_on_synthetic_planes_fraction'] = float(np.mean(errors < .1))
        report.update(checks=checks, odometry_messages=len(poses), cloud_messages=len(clouds), passed=all(checks.values()))
    except Exception as exc:
        report.update(passed=False, error=repr(exc))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    report['launch_exit_code'] = process.returncode
    report['passed'] = report['passed'] and process.returncode == 0
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
