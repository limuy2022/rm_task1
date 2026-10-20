#!/usr/bin/env python3
"""Gazebo 三维雷达/IMU → 导航压缩包原版 Point-LIO → 真实点云建图。

Gazebo 真值仅用于评分和运动阶段记录，不发布为定位输入。多次瞬时扫描按
各自真实采样时间组成 Livox 消息；不伪造帧内连续扫描或 MID360 扫描模式。
在根 Dockerfile 的 gazebo-sim 目标中构建并运行，所有话题在 /sim/gazebo。
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from PIL import Image, ImageDraw
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from livox_ros_driver2.msg import CustomMsg, CustomPoint

from gimbal_common.qos_utils import latched_qos, sensor_qos
from grid_mapper.cloud_grid import pointcloud2_xyz
from grid_mapper.pgm_io import read_pgm
from gazebo.sensor_config import use_si_acceleration

ROOT = Path(__file__).resolve().parents[1]
NS = '/sim/gazebo'
RESOLUTION = .05
ORIGIN = np.array([-5., -4.])
INITIAL_BODY = np.array([0., 0., .7])


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def envelope(t, duration=4., ramp=.5):
    if t < 0 or t >= duration: return 0.
    edge = min(t, duration-t)
    return .5-.5*math.cos(math.pi*min(edge/ramp, 1.))


class GazeboLio(Node):
    def __init__(self, output, timeout, real_time_factor, record_bag, imu_mode, scan_lines):
        super().__init__('gazebo_lio_trial', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.output, self.timeout = output, timeout
        self.processes, self.logs = [], []
        self.truth, self.estimates, self.imus, self.statuses = [], [], [], []
        self.cloud_batches, self.batch_stamps = [], []
        self.measurement_samples = []
        self.scan_lines = scan_lines
        self.published_frames, self.published_points = 0, 0
        self.raw_scans, self.raw_bad, self.offset_spans = 0, 0, []
        self.latest_map = None
        self.motion_start, self.last_command = None, -1.
        self.last_raw_stamp, self.last_truth_stamp, self.last_estimate_stamp = -1, -1, -1
        self.out_of_order = {'raw_cloud': 0, 'truth': 0, 'estimate': 0, 'imu': 0}
        self.cmd_pub = self.create_publisher(Twist, NS+'/cmd_vel', 10)
        self.lidar_pub = self.create_publisher(CustomMsg, NS+'/livox/lidar', sensor_qos(10))
        self.create_subscription(PointCloud2, NS+'/lidar/points', self.on_cloud, sensor_qos(100))
        self.create_subscription(Imu, NS+'/imu', self.on_imu, sensor_qos(1000))
        self.create_subscription(PoseArray, NS+'/truth', self.on_truth, sensor_qos(1000))
        self.create_subscription(Odometry, NS+'/lio/aft_mapped_to_init', self.on_estimate, sensor_qos(1000))
        self.create_subscription(OccupancyGrid, NS+'/map', self.on_map, latched_qos())
        self.create_subscription(String, NS+'/mapping_status', lambda m: self.statuses.append(json.loads(m.data)), 100)
        self.save_client = self.create_client(Trigger, NS+'/save_map')
        self.mapper_parameters = self.create_client(SetParameters, '/cloud_mapper/set_parameters')
        config = yaml.safe_load((ROOT/'src/gimbal_tracker_bringup/config/point_lio_mid360.yaml').read_text())
        params = config['/**']['ros__parameters']
        params['use_sim_time'] = True
        params['use_imu_as_input'] = imu_mode == 'input'
        params['common'].update(lid_topic=NS+'/livox/lidar', imu_topic=NS+'/imu')
        # 该版本饱和检查直接比较原始 IMU 消息；把输入从原配置单位改成
        # m/s² 时，必须同步换算饱和阈值，否则9.81的静止重力会超过3。
        use_si_acceleration(params['mapping'])
        params['mapping']['imu_time_inte'] = .005
        params['mapping']['lidar_time_inte'] = .1
        params['preprocess']['scan_line'] = scan_lines
        params['point_filter_num'] = 2
        params['filter_size_surf'] = .15
        params['filter_size_map'] = .15
        params['publish']['tf_send_en'] = False
        self.lidar_offset = params['mapping']['extrinsic_T']
        (output/'point_lio.yaml').write_text(yaml.safe_dump(config, allow_unicode=True))
        tree = ET.parse(ROOT/'tools/gazebo/lio_room.sdf')
        tree.find('.//physics/real_time_factor').text = str(real_time_factor)
        tree.find('.//sensor[@name="lidar"]/lidar/scan/vertical/samples').text = str(scan_lines)
        tree.find(".//model[@name='lidar_rig']/link/sensor[@name='lidar']/pose").text = ' '.join(map(str, self.lidar_offset))+' 0 0 0'
        tree.write(output/'world.sdf', encoding='unicode', xml_declaration=True)
        self.record_bag = record_bag

    def on_map(self, msg):
        self.latest_map = msg

    def on_imu(self, msg):
        t = seconds(msg.header.stamp)
        if self.imus and t <= self.imus[-1][0]:
            self.out_of_order['imu'] += 1
            return
        a, w = msg.linear_acceleration, msg.angular_velocity
        self.imus.append([t, a.x, a.y, a.z, w.x, w.y, w.z])

    def on_truth(self, msg):
        if len(msg.poses) != 1:
            raise ValueError(f'真值话题必须仅含传感器平台模型，实际 {len(msg.poses)} 个 pose')
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= self.last_truth_stamp:
            self.out_of_order['truth'] += 1
            return
        self.last_truth_stamp = stamp
        p = msg.poses[0]
        self.truth.append([stamp*1e-9, p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)])

    def on_estimate(self, msg):
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
            raise ValueError('Point-LIO 输出 frame 与生产配置不一致')
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= self.last_estimate_stamp:
            self.out_of_order['estimate'] += 1
            return
        self.last_estimate_stamp = stamp
        p = msg.pose.pose
        row = [stamp*1e-9, p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)]
        if not np.isfinite(row).all(): raise ValueError('Point-LIO 输出非有限位姿')
        self.estimates.append(row)

    def on_cloud(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= self.last_raw_stamp:
            self.out_of_order['raw_cloud'] += 1
            return
        self.last_raw_stamp = stamp
        self.raw_scans += 1
        # Gazebo 的一帧是同一采样时刻的快照。保留该时刻供批内所有回波使用，
        # 只在合并不同快照时产生 offset_time，不插值或重新投影任何测量。
        xyz = pointcloud2_xyz(msg)
        if not len(xyz):
            self.raw_bad += 1
            return
        if self.raw_scans % 4 == 0:
            self.measurement_samples.append((stamp*1e-9, xyz[np.linspace(0, len(xyz)-1, 100, dtype=int)].copy()))
        self.cloud_batches.append(xyz)
        self.batch_stamps.append(stamp)
        if len(self.cloud_batches) < 4: return
        base = self.batch_stamps[0]
        out = CustomMsg()
        out.header.frame_id = 'livox_frame'
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(base, 1_000_000_000)
        out.timebase, out.lidar_id = base, 0
        points = []
        for frame, t in zip(self.cloud_batches, self.batch_stamps):
            offset = t-base
            if not 0 <= offset < 150_000_000:
                raise ValueError('Gazebo 扫描时间存在超大间隔，不能拼成正常 Livox 帧')
            for i, xyz in enumerate(frame):
                point = CustomPoint()
                point.x, point.y, point.z = map(float, xyz)
                point.offset_time = offset
                # line 供通道范围过滤；去除无效回波后不宣称保留实物通道。
                point.line = (i//180) % self.scan_lines
                point.tag, point.reflectivity = 0x10, 100
                points.append(point)
        out.point_num, out.points = len(points), points
        self.lidar_pub.publish(out)
        self.offset_spans.append(self.batch_stamps[-1]-base)
        self.published_frames += 1
        self.published_points += len(points)
        self.cloud_batches.clear()
        self.batch_stamps.clear()

    def launch(self, name, command, env=None):
        log = (self.output/(name+'.log')).open('w')
        self.logs.append(log)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                env=env, start_new_session=True)
        self.processes.append((name, proc))

    def launch_ros(self, name, package, executable, params, remaps):
        config = self.output/(name+'.yaml')
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'use_sim_time': True, **params}}}, allow_unicode=True))
        command = ['ros2', 'run', package, executable, '--ros-args', '--params-file', str(config),
                   '-r', '__node:='+name, '-r', '/tf:='+NS+'/tf', '-r', '/tf_static:='+NS+'/tf_static']
        for source, target in remaps.items(): command.extend(['-r', source+':='+target])
        self.launch(name, command)

    def start(self):
        # 估计器先启动，避免遗漏启动静止窗口。真值话题从未接到估计器。
        self.launch('point_lio', ['ros2', 'run', 'point_lio', 'pointlio_mapping', '--ros-args',
                    '--params-file', str(self.output/'point_lio.yaml'), '-r', '__ns:='+NS+'/lio'])
        self.launch_ros('cloud_mapper', 'grid_mapper', 'cloud_mapping_node', {
            'cloud_topic': NS+'/lio/cloud_registered', 'odom_topic': NS+'/lio/aft_mapped_to_init',
            'origin_model_confirmed': True, 'sensor_origin_in_body': self.lidar_offset,
            'floor_z': -.7, 'floor_confirmed': True, 'resolution': RESOLUTION,
            'origin_xy': ORIGIN.tolist(), 'width_m': 10., 'height_m': 8.,
            'publish_hz': 2., 'autosave_seconds': 0., 'output_dir': str(self.output/'maps'),
        }, {'/team47/slam_map': NS+'/map', '/team47/cloud_mapping/status': NS+'/mapping_status',
            '/team47/save_slam_map': NS+'/save_map'})
        # 方括号是 Gazebo→ROS，右方括号是 ROS→Gazebo；不桥接硬件话题。
        self.launch('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    NS+'/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                    NS+'/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    '/model/lidar_rig/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V',
                    NS+'/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    '--ros-args', '-r', '/model/lidar_rig/pose:='+NS+'/truth'])
        env = os.environ.copy()
        env.pop('DISPLAY', None)
        self.launch('gazebo', ['ign', 'gazebo', '-s', '-r', '--headless-rendering', '-v', '3', str(self.output/'world.sdf')], env)
        if self.record_bag:
            self.launch('bag', ['ros2', 'bag', 'record', '-o', str(self.output/'sensor_bag'),
                        '/clock', NS+'/livox/lidar', NS+'/imu', NS+'/truth',
                        NS+'/lio/aft_mapped_to_init', NS+'/lio/cloud_registered'])

    def drive(self, now):
        cmd = Twist()
        if self.motion_start is not None:
            elapsed = now-self.motion_start
            if 0 <= elapsed < 32.:
                phase = elapsed % 8.
                if phase < 4.:
                    cmd.linear.x = .3*envelope(phase)
                else:
                    cmd.angular.z = (math.pi/2/3.5)*envelope(phase-4.)
        self.cmd_pub.publish(cmd)

    def pump(self):
        rclpy.spin_once(self, timeout_sec=.01)
        for name, proc in self.processes:
            if proc.poll() is not None:
                raise RuntimeError(f'{name} 提前退出，code={proc.returncode}；见 {name}.log')

    def run(self):
        self.start()
        wall_start = time.monotonic()
        last_progress = -1
        while time.monotonic()-wall_start < self.timeout:
            self.pump()
            now = self.get_clock().now().nanoseconds*1e-9
            if now-self.last_command >= .02:
                self.drive(now)
                self.last_command = now
            if self.motion_start is None and len(self.estimates) >= 20 and len(self.truth) >= 100:
                self.motion_start = now+3.
                print(f'Point-LIO 已输出有效位姿；运动起点 sim_t={self.motion_start:.3f}s', flush=True)
            if now > 15 and not self.estimates:
                raise RuntimeError(f'15秒仿真时间后仍无 Point-LIO 位姿；raw={self.raw_scans} custom={self.published_frames} imu={len(self.imus)} truth={len(self.truth)}')
            progress = int(now//5)
            if progress != last_progress and now > 0:
                print(f'sim_t={now:.1f}s raw={self.raw_scans} custom={self.published_frames} lio={len(self.estimates)} truth={len(self.truth)}', flush=True)
                last_progress = progress
            if self.motion_start is not None and now >= self.motion_start+35.:
                self.drive(self.motion_start+36.)
                break
        else:
            raise RuntimeError('仿真超过墙钟超时；保留传感器统计和日志')
        # 暂停物理世界后排空传感器/估计器队列，避免服务保存与地图定时发布
        # 恰好落在两帧之间，导致报告、PGM 和订阅地图对应不同快照。
        pause = subprocess.run(['ign', 'service', '-s', '/world/lio_room/control',
                    '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '2000', '--req', 'pause: true'], capture_output=True, text=True, timeout=5.)
        (self.output/'pause.log').write_text(pause.stdout+pause.stderr)
        if pause.returncode or 'true' not in pause.stdout: raise RuntimeError('暂停 Gazebo 失败')
        deadline = time.monotonic()+.6
        while time.monotonic() < deadline: self.pump()
        # 只在物理输入已经暂停后切换本次地图发布定时器到墙钟，使最终状态
        # 可以发布；外部测量的 header.stamp 始终保留原仿真时刻。
        request = SetParameters.Request()
        request.parameters = [Parameter('use_sim_time', value=False).to_parameter_msg()]
        future = self.mapper_parameters.call_async(request)
        deadline = time.monotonic()+5.
        while not future.done() and time.monotonic() < deadline: self.pump()
        if not future.done() or not all(r.successful for r in future.result().results):
            raise RuntimeError('最终地图发布时钟切换失败')
        deadline = time.monotonic()+.7
        while time.monotonic() < deadline: self.pump()
        if not self.save_client.service_is_ready(): raise RuntimeError('地图保存服务未就绪')
        future = self.save_client.call_async(Trigger.Request())
        deadline = time.monotonic()+10.
        while not future.done() and time.monotonic() < deadline: self.pump()
        if not future.done(): raise RuntimeError('地图保存超时')
        if not future.result().success: raise RuntimeError(future.result().message)
        return self.evaluate(time.monotonic()-wall_start)

    def evaluate(self, elapsed_wall):
        gt, estimate = np.asarray(self.truth), np.asarray(self.estimates)
        if len(gt) < 100 or len(estimate) < 100 or self.latest_map is None:
            raise RuntimeError('没有足够的定位、真值或地图输出')
        valid = (estimate[:, 0] >= gt[0, 0]) & (estimate[:, 0] <= gt[-1, 0])
        estimate = estimate[valid]
        matched = np.column_stack([np.interp(estimate[:, 0], gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
        matched -= INITIAL_BODY
        expected_yaw = np.interp(estimate[:, 0], gt[:, 0], np.unwrap(gt[:, 4]))
        position_error = np.linalg.norm(estimate[:, 1:4]-matched, axis=1)
        yaw_error = np.abs(np.arctan2(np.sin(estimate[:, 4]-expected_yaw), np.cos(estimate[:, 4]-expected_yaw)))
        static = estimate[:, 0] < self.motion_start
        last = estimate[:, 0] > self.motion_start+33.
        truth_last = gt[gt[:, 0] > self.motion_start+33.]
        msg = self.latest_map
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        wall_coverage = []
        for axis, wall, lo, hi in ((0, -4., -3., 3.), (0, 4., -3., 3.), (1, -3., -4., 4.), (1, 3., -4., 4.)):
            for v in np.arange(lo+.15, hi-.1, .05):
                xy = [wall, v] if axis == 0 else [v, wall]
                i, j = np.floor((np.array(xy)-ORIGIN)/RESOLUTION).astype(int)
                wall_coverage.append(bool(np.any(grid[j-2:j+3, i-2:i+3] == 100)))
        outside = []
        for y in np.arange(-3.8, 3.8, .1):
            for x in np.arange(-4.8, 4.8, .1):
                if abs(x) > 4.2 or abs(y) > 3.2:
                    i, j = np.floor((np.array([x, y])-ORIGIN)/RESOLUTION).astype(int)
                    outside.append(grid[j, i] == -1)
        imu = np.array(self.imus)
        static_imu = imu[imu[:, 0] < self.motion_start]
        if not np.any(static) or not np.any(last) or not len(truth_last) or not len(static_imu):
            raise RuntimeError('静止或结束窗口缺少测量，不能计算完整误差')
        acceleration_norm = np.linalg.norm(static_imu[:, 1:4], axis=1)
        # 用场景真值独立核验模拟测量本身，避免错误的传感器模型被误判为
        # 定位算法故障。这里只评价测量，从未用真值修改定位输入。
        world_velocity = np.gradient(gt[:, 1:4], gt[:, 0], axis=0)
        world_acceleration = np.gradient(world_velocity, gt[:, 0], axis=0)
        ia = np.column_stack([np.interp(imu[:, 0], gt[:, 0], world_acceleration[:, k]) for k in range(3)])
        iyaw = np.interp(imu[:, 0], gt[:, 0], np.unwrap(gt[:, 4]))
        c, s = np.cos(iyaw), np.sin(iyaw)
        expected_acceleration = np.column_stack([c*ia[:, 0]+s*ia[:, 1], -s*ia[:, 0]+c*ia[:, 1], ia[:, 2]+9.81])
        imu_consistency_rmse = float(np.sqrt(np.mean(np.sum((expected_acceleration-imu[:, 1:4])**2, axis=1))))
        boxes = []
        for model in ET.parse(self.output/'world.sdf').findall('.//world/model'):
            if model.findtext('static') != 'true': continue
            size = model.find('.//visual/geometry/box/size')
            if size is None: continue
            center = np.array(list(map(float, model.findtext('pose').split())))[:3]
            boxes.append((center, np.array(list(map(float, size.text.split())))/2))
        surface_errors = []
        for t, local in self.measurement_samples:
            if not gt[0, 0] <= t <= gt[-1, 0]: continue
            yaw = np.interp(t, gt[:, 0], np.unwrap(gt[:, 4]))
            c, s = math.cos(yaw), math.sin(yaw)
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
            p = np.array([np.interp(t, gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
            world = (local+np.array(self.lidar_offset))@R.T+p
            distances = []
            for center, half in boxes:
                d = np.abs(world-center)-half
                distances.append(np.abs(np.linalg.norm(np.maximum(d, 0), axis=1)+np.minimum(np.max(d, axis=1), 0)))
            surface_errors.extend(np.min(distances, axis=0))
        disk, _ = read_pgm(str(self.output/'maps/47_slam_map.pgm'))
        expected_pixels = np.where(grid == 100, 0, np.where(grid == 0, 254, 205)).astype(np.uint8)[::-1]
        metrics = {
            'position_rmse_m': float(np.sqrt(np.mean(position_error**2))),
            'position_max_error_m': float(np.max(position_error)),
            'yaw_max_error_rad': float(np.max(yaw_error)),
            'static_position_max_error_m': float(np.max(position_error[static])),
            'estimated_return_error_m': float(np.linalg.norm(np.mean(estimate[last, 1:4], axis=0))),
            'truth_return_error_m': float(np.linalg.norm(np.mean(truth_last[:, 1:4], axis=0)-INITIAL_BODY)),
            'truth_path_extent_m': float(np.max(np.ptp(gt[:, 1:3], axis=0))),
            'wall_coverage_within_0_1m': float(np.mean(wall_coverage)),
            'outside_unknown_fraction': float(np.mean(outside)),
            'static_imu_acceleration_norm_mean': float(np.mean(acceleration_norm)),
            'dynamic_imu_horizontal_acceleration_peak': float(np.max(np.linalg.norm(imu[:, 1:3], axis=1))),
            'imu_truth_acceleration_rmse_mps2': imu_consistency_rmse,
            'raw_returns_scene_distance_p95_m': float(np.percentile(surface_errors, 95)),
            'raw_scans': self.raw_scans, 'livox_frames': self.published_frames,
            'livox_points': self.published_points, 'imu_messages': len(self.imus),
            'truth_messages': len(self.truth), 'estimated_poses': len(self.estimates),
            'subscan_offset_min_ns': min(self.offset_spans), 'subscan_offset_max_ns': max(self.offset_spans),
            'wall_seconds': elapsed_wall, 'simulation_end_s': float(gt[-1, 0]),
            'out_of_order_messages': self.out_of_order,
        }
        checks = {
            'trajectory_actually_moved': metrics['truth_path_extent_m'] > .8,
            'truth_returned_to_start': metrics['truth_return_error_m'] < .05,
            'position_rmse': metrics['position_rmse_m'] < .15,
            'position_max': metrics['position_max_error_m'] < .35,
            'yaw_max': metrics['yaw_max_error_rad'] < .15,
            'static_drift': metrics['static_position_max_error_m'] < .05,
            'estimated_return': metrics['estimated_return_error_m'] < .2,
            'wall_coverage': metrics['wall_coverage_within_0_1m'] > .85,
            'outside_unknown': metrics['outside_unknown_fraction'] > .98,
            'imu_specific_force_units': abs(metrics['static_imu_acceleration_norm_mean']-9.81) < .1,
            'imu_reports_motion_acceleration': metrics['dynamic_imu_horizontal_acceleration_peak'] > .4,
            'imu_matches_gazebo_motion': imu_consistency_rmse < .1,
            'raw_returns_match_scene': metrics['raw_returns_scene_distance_p95_m'] < .04,
            'real_subscan_timestamps': min(self.offset_spans) > 0 and max(self.offset_spans) < 150_000_000,
            'message_order': not any(self.out_of_order.values()),
            'map_pgm_matches_ros': np.array_equal(disk, expected_pixels),
        }
        with (self.output/'trajectory.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'truth_x', 'truth_y', 'truth_z', 'estimated_x', 'estimated_y', 'estimated_z', 'truth_yaw', 'estimated_yaw', 'position_error_m', 'yaw_error_rad'])
            for i, row in enumerate(estimate): writer.writerow([row[0], *matched[i], *row[1:4], expected_yaw[i], row[4], position_error[i], yaw_error[i]])
        for name, rows in (('gazebo_truth.csv', self.truth), ('point_lio_odometry.csv', self.estimates), ('imu.csv', self.imus)):
            with (self.output/name).open('w', newline='') as f: csv.writer(f).writerows(rows)
        self.preview(disk, gt[:, 1:3], estimate[:, 1:3], metrics)
        return {'passed': all(checks.values()), 'metrics': metrics, 'checks': checks}

    def preview(self, disk, gt_xy, estimated_xy, metrics):
        scale = 5
        image = Image.fromarray(disk).convert('RGB').resize((disk.shape[1]*scale, disk.shape[0]*scale), Image.NEAREST)
        draw = ImageDraw.Draw(image)
        def pixel(xy):
            x, y = (np.array(xy)-ORIGIN)/RESOLUTION
            return (float(x*scale), float((disk.shape[0]-y)*scale))
        for a, b in (((-4, -3), (4, -3)), ((4, -3), (4, 3)), ((4, 3), (-4, 3)), ((-4, 3), (-4, -3))):
            draw.line([pixel(a), pixel(b)], fill=(30, 120, 240), width=2)
        draw.line([pixel(xy) for xy in gt_xy], fill=(20, 170, 60), width=4)
        draw.line([pixel(xy) for xy in estimated_xy], fill=(230, 60, 40), width=2)
        draw.rectangle((8, 8, 850, 64), fill='white')
        draw.text((16, 14), 'GAZEBO 3-D LIDAR + IMU -> REAL POINT-LIO -> PRODUCTION CLOUD MAPPER', fill='black')
        draw.text((16, 32), 'Blue: true walls / Green: Gazebo truth / Red: Point-LIO estimate', fill='black')
        draw.text((16, 48), f'Position RMSE: {metrics["position_rmse_m"]:.4f} m / max: {metrics["position_max_error_m"]:.4f} m', fill='black')
        image.save(self.output/'overview.png')

    def close(self):
        self.cmd_pub.publish(Twist())
        codes = {}
        for name, proc in reversed(self.processes):
            if proc.poll() is None:
                try: os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError: pass
        for name, proc in reversed(self.processes):
            try: proc.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3.)
            codes[name] = proc.returncode
        for log in self.logs: log.close()
        self.destroy_node()
        return codes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=600.)
    parser.add_argument('--real-time-factor', type=float, default=.5)
    parser.add_argument('--record-bag', action='store_true')
    parser.add_argument('--imu-mode', choices=['input', 'output'], default='input',
                        help='仿真默认使用已验收的 input 模式；不修改实车配置')
    parser.add_argument('--scan-lines', type=int, choices=[16, 32], default=16,
                        help='同时修改 Gazebo 光束数和 Point-LIO 预处理配置')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('必须设置 ROS_LOCALHOST_ONLY=1 和独立 ROS_DOMAIN_ID')
    if not os.environ.get('IGN_PARTITION'): parser.error('必须设置独立 IGN_PARTITION')
    if not 0 < args.real_time_factor <= 1 or not 30 <= args.timeout <= 3600:
        parser.error('real-time-factor 须在 (0,1]，timeout 须为 30～3600 秒')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    app = GazeboLio(output, args.timeout, args.real_time_factor, args.record_bag, args.imu_mode, args.scan_lines)
    report = {'kind': 'gazebo_raw_lidar_imu_point_lio', 'ground_truth_used_as_odometry_input': False,
              'hardware_accessed': False, 'imu_mode': args.imu_mode, 'scan_lines': args.scan_lines,
              'source_archive_sha256': hashlib.sha256((ROOT/'archives/robomaster_navigation.tar.gz').read_bytes()).hexdigest(),
              'limitations': ['悬浮刚性平台，不模拟轮胎或悬架', '规则多线雷达，不是MID360非重复扫描模式',
                              '当前场景没有额外注入传感器噪声、偏置或时延',
                              '每次瞬时快照内部时间相同；四次快照按各自采样时刻组成一帧',
                              '仿真误差门槛不是官方评分或实机精度保证', '没有运行云台视觉或九列提交']}
    try:
        report.update(app.run())
    except Exception as exc:
        report.update(passed=False, error=repr(exc), raw_scans=app.raw_scans,
                      livox_frames=app.published_frames, imu_messages=len(app.imus),
                      truth_messages=len(app.truth), estimated_poses=len(app.estimates))
        (output/'failure.log').write_text(traceback.format_exc())
    finally:
        report['node_exit_codes'] = app.close()
        if rclpy.ok(): rclpy.shutdown()
    report['clean_shutdown'] = all(code == 0 for code in report['node_exit_codes'].values())
    report['passed'] = report['passed'] and report['clean_shutdown']
    report['Gazebo_executed'] = app.raw_scans > 0 and len(app.truth) > 0
    report['Point_LIO_executed'] = len(app.estimates) > 0
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
