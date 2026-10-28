#!/usr/bin/env python3
"""仅在隔离容器验证比赛机安装包：发布合成雷达/IMU，不连接硬件。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('仅允许在ROS_LOCALHOST_ONLY=1及独立ROS_DOMAIN_ID的测试容器运行')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('team43_lio_backend'))
    params = yaml.safe_load((share/'config/small_point_lio_mid360.yaml').read_text())
    config = params['/**']['ros__parameters']
    config.update(acc_norm=9.81, satu_acc=29.43, extrinsic_T=[.1, -.02, .04])
    path = output/'test_params.yaml'
    path.write_text(yaml.safe_dump(params))
    rclpy.init()
    node = Node('team43_lio_install_test')
    poses, clouds = [], []
    node.create_subscription(Odometry, '/lio_install_test/aft_mapped_to_init', poses.append, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/lio_install_test/cloud_registered', clouds.append, qos_profile_sensor_data)
    lidar = node.create_publisher(CustomMsg, '/lio_install_test/lidar', qos_profile_sensor_data)
    imu = node.create_publisher(Imu, '/lio_install_test/imu', qos_profile_sensor_data)
    points = []
    for a in range(-10, 11):
        for b in range(-6, 7):
            for xyz in ((2., a*.15, b*.15), (a*.15, 2., b*.15), (a*.15, b*.15, -1.)):
                p = CustomPoint()
                p.x, p.y, p.z = xyz
                p.tag = 0x10
                p.reflectivity = 100
                points.append(p)
    log = (output/'launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'team43_lio_backend', 'lio.launch.py',
                               'namespace:=lio_install_test', 'lidar_topic:=/lio_install_test/lidar',
                               'imu_topic:=/lio_install_test/imu', 'params_file:='+str(path)],
                              cwd=output, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    report = {'hardware_accessed': False, 'package_share': str(share)}
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
                msg = CustomMsg()
                msg.header.stamp = stamp.to_msg()
                msg.timebase = stamp.nanoseconds
                msg.points = points
                msg.point_num = len(points)
                lidar.publish(msg)
                next_lidar = now+.1
            rclpy.spin_once(node, timeout_sec=.001)
            if now-start > 5. and len(poses) >= 20 and len(clouds) >= 10:
                break
        stamps = [p.header.stamp.sec+p.header.stamp.nanosec*1e-9 for p in poses]
        checks = {
            'odometry_received': len(poses) >= 20,
            'cloud_received': len(clouds) >= 10,
            'body_frame': bool(poses) and all(p.header.frame_id == 'camera_init' and p.child_frame_id == 'body' for p in poses),
            'cloud_world_frame': bool(clouds) and all(c.header.frame_id == 'camera_init' and c.width > 0 for c in clouds),
            'finite_static_body_pose': bool(poses) and all(math.isfinite(v) and abs(v) < .05 for p in poses
                for v in (p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z)),
            'timestamps_increase': len(stamps) > 1 and all(b > a for a, b in zip(stamps, stamps[1:])),
        }
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
