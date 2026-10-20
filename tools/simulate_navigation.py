#!/usr/bin/env python3
"""已知房间与轨迹的 ROS 2 建图集成仿真；不模拟 Point-LIO 原始测量。

须在根 Dockerfile 的 Humble 容器中构建、source 后运行。四个生产节点
各自在独立进程运行，输入/输出走真实 DDS。所有话题隔离到 /sim，产物
不等同于实机精度、Point-LIO 精度或正式评分文件。
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

import numpy as np
import yaml
from PIL import Image, ImageDraw
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from gimbal_common.qos_utils import latched_qos, sensor_qos
from grid_mapper.pgm_io import read_pgm

ROOT = Path(__file__).resolve().parents[1]
PREFIX = '/sim/navigation'
MOUNT_XYZ = np.array([0.31, -0.19, 0.48])
MOUNT_RPY = [0.09, -0.06, 0.12]
GRID_ORIGIN = np.array([-4., -3.])
RESOLUTION = 0.05


def rotation(rpy):
    """仿真自己的欧拉角公式，不调用待测坐标转换器。"""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def quaternion(rpy):
    r, p, y = np.array(rpy)/2
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                     cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


def multiply_quaternion(a, b):
    av, bv = a[:3], b[:3]
    return np.r_[a[3]*bv+b[3]*av+np.cross(av, bv), a[3]*b[3]-av@bv]


def set_stamp(stamp, value):
    total = round(value*1_000_000_000)
    stamp.sec, stamp.nanosec = divmod(total, 1_000_000_000)


def stamp_key(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


def truth(index, scans):
    # 前五帧静止，随后整圈运动并回到初始点；非零起点/yaw、全三维杆臂。
    phase = 2*math.pi*max(0, index-5)/(scans-6)
    xyz = np.array([0.35+0.7*math.cos(phase), 0.4+0.55*math.sin(phase), 0.])
    rpy = [0.12*math.sin(phase), 0.09*math.sin(2*phase), 0.7+phase]
    return xyz, rpy


def room_scan(origin, yaw, rng, noise, person):
    """射线只返回最近墙面/临时行人；墙外不伪造空闲证据。

    这是带高度的二维射线场景，不宣称复现 MID360 扫描模式或运动畸变。
    """
    angles = yaw+np.linspace(-math.pi, math.pi, 720, endpoint=False)
    directions = np.column_stack([np.cos(angles), np.sin(angles)])
    distances = np.full(len(angles), np.inf)
    for axis, wall in ((0, -3.), (0, 4.), (1, -2.), (1, 3.)):
        valid_axis = np.abs(directions[:, axis]) > 1e-10
        candidate = np.full(len(angles), np.inf)
        candidate[valid_axis] = (wall-origin[axis])/directions[valid_axis, axis]
        # 非轴向平行的射线才参加边界判断，避免 inf*0。
        hit = origin[:2]+np.where(np.isfinite(candidate), candidate, 0)[:, None]*directions
        valid = valid_axis & (candidate > 0.5) & (hit[:, 0] >= -3.-1e-8) & (hit[:, 0] <= 4.+1e-8) & (hit[:, 1] >= -2.-1e-8) & (hit[:, 1] <= 3.+1e-8)
        distances = np.minimum(distances, np.where(valid, candidate, np.inf))
    if person:
        rel = origin[:2]-np.array([-0.65, 0.45])
        b = 2*(directions@rel)
        disc = b*b-4*(rel@rel-0.22**2)
        near = (-b-np.sqrt(np.maximum(0., disc)))/2
        distances = np.minimum(distances, np.where((disc >= 0) & (near > 0.5), near, np.inf))
    valid = np.isfinite(distances)
    distances = distances[valid]+rng.normal(0., noise, np.count_nonzero(valid))
    xy = origin[:2]+distances[:, None]*directions[valid]
    # 两层墙面回波及超出障碍高度的天花板点，检查高度过滤。
    return np.vstack([np.column_stack([xy, np.full(len(xy), z)]) for z in (0.35, 1.1, 2.5)])


class Simulation(Node):
    def __init__(self, output, noise, scans, seed, inject_mount_error=False):
        super().__init__('navigation_simulator')
        self.output, self.noise, self.scans = output, noise, scans
        self.rng = np.random.default_rng(seed)
        self.inject_mount_error = inject_mount_error
        self.processes, self.logs = [], []
        self.statuses, self.raw_poses, self.map_poses = [], {}, {}
        self.source_map, self.final_map = None, None
        self.expected, self.pose_rows = {}, []
        self.now = 100.
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, PREFIX+'/lio_odom', sensor_qos(10))
        self.cloud_pub = self.create_publisher(PointCloud2, PREFIX+'/cloud_registered', sensor_qos(5))
        self.start_pub = self.create_publisher(Header, PREFIX+'/replay_start', latched_qos())
        self.create_subscription(String, PREFIX+'/status', lambda m: self.statuses.append(json.loads(m.data)), 50)
        self.create_subscription(Odometry, PREFIX+'/base_pose_raw', lambda m: self.raw_poses.__setitem__(stamp_key(m.header.stamp), m), sensor_qos(100))
        self.create_subscription(Odometry, PREFIX+'/base_pose', lambda m: self.map_poses.__setitem__(stamp_key(m.header.stamp), m), sensor_qos(100))
        self.create_subscription(OccupancyGrid, PREFIX+'/source_map', self.on_source_map, latched_qos())
        self.create_subscription(OccupancyGrid, PREFIX+'/map', self.on_final_map, latched_qos())
        self.save_client = self.create_client(Trigger, PREFIX+'/save_map')
        self.floor_client = self.create_client(SetParameters, PREFIX+'/cloud_mapper/set_parameters')
        # 合成 camera_init 与 odom 约定同轴同原点；真实安装值不能照搬。
        self.tf = StaticTransformBroadcaster(self, qos=latched_qos())
        cfg = yaml.safe_load((ROOT/'src/gimbal_tracker_bringup/config/point_lio_mid360.yaml').read_text())
        self.lidar_offset = np.array(cfg['/**']['ros__parameters']['mapping']['extrinsic_T'])

    def on_source_map(self, message):
        self.source_map = message

    def on_final_map(self, message):
        self.final_map = message

    def pump(self, seconds=0.04):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.01, max(0., deadline-time.monotonic())))
        for name, proc in self.processes:
            if proc.poll() is not None:
                raise RuntimeError(f'{name} 提前退出，code={proc.returncode}；检查同目录日志')

    def wait_for(self, condition, reason, timeout=8.):
        deadline = time.monotonic()+timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise RuntimeError('等待超时：'+reason)
            # 等待期间也推进仿真时间，否则刚收到位姿的生产定时器永远
            # 等不到下一拍；保持时间单调，禁止回退重放。
            self.publish_clock(round(self.now+0.02, 9))
            self.pump()

    def publish_clock(self, value):
        self.now = value
        msg = Clock()
        set_stamp(msg.clock, value)
        self.clock_pub.publish(msg)

    def advance(self, seconds):
        for _ in range(max(1, round(seconds/0.02))):
            self.publish_clock(round(self.now+0.02, 9))
            self.pump(0.012)

    def launch(self, package, executable, name, parameters, remaps):
        config = self.output/(name+'.yaml')
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'use_sim_time': True, **parameters}}}, allow_unicode=True))
        log = (self.output/(name+'.log')).open('w')
        self.logs.append(log)
        command = ['ros2', 'run', package, executable, '--ros-args', '--params-file', str(config), '-r', '__node:='+name,
                   '-r', '/tf:='+PREFIX+'/tf', '-r', '/tf_static:='+PREFIX+'/tf_static']
        for source, target in remaps.items():
            command.extend(['-r', source+':='+target])
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append((name, proc))

    def start(self):
        self.launch('grid_mapper', 'cloud_mapping_node', 'cloud_mapper', {
            'cloud_topic': PREFIX+'/cloud_registered', 'odom_topic': PREFIX+'/lio_odom',
            'origin_model_confirmed': True, 'sensor_origin_in_body': self.lidar_offset.tolist(),
            'width_m': 10., 'height_m': 8., 'origin_xy': GRID_ORIGIN.tolist(),
            'floor_z': 0., 'floor_confirmed': False, 'publish_hz': 5., 'autosave_seconds': 0.,
            'output_dir': str(self.output/'source'),
        }, {'/team47/slam_map': PREFIX+'/source_map', '/team47/cloud_mapping/status': PREFIX+'/status',
            '/team47/save_slam_map': PREFIX+'/save_map', '/cloud_mapper/set_parameters': PREFIX+'/cloud_mapper/set_parameters'})
        mount = MOUNT_XYZ.copy()
        if self.inject_mount_error:
            mount[0] += 0.25
        self.launch('gimbal_localization', 'base_pose_provider_node', 'base_provider', {
            'localization.base_pose_source': 'odom', 'localization.odom_topic': PREFIX+'/lio_odom',
            'localization.odom_frame_id': 'camera_init', 'localization.odom_child_frame_id': 'body',
            'localization.sensor_in_base_xyz': mount.tolist(), 'localization.sensor_in_base_rpy': MOUNT_RPY,
            'localization.sensor_extrinsics_confirmed': True, 'localization.parent_transform_confirmed': True,
            'localization.odom_twist_source': 'pose_difference',
        }, {'/gimbal/base_pose_raw': PREFIX+'/base_pose_raw', '/judge/replay_start': PREFIX+'/provider_reset'})
        self.launch('gimbal_localization', 'map_frame_broadcaster_node', 'map_origin', {}, {
            '/gimbal/base_pose_raw': PREFIX+'/base_pose_raw', '/gimbal/base_pose': PREFIX+'/base_pose',
            '/judge/replay_start': PREFIX+'/replay_start', '/gimbal/origin_freeze': PREFIX+'/origin_freeze',
            '/gimbal/t_map_world': PREFIX+'/t_map_world'})
        self.launch('grid_mapper', 'grid_mapper_node', 'map_exporter', {
            'grid.map_source': 'topic', 'grid.map_topic': PREFIX+'/source_map',
            'grid.source_frame': 'camera_init', 'grid.out_dir': str(self.output/'competition'),
            'grid.output_basename': '47_map', 'grid.write_preview_png': False,
        }, {'/map/grid': PREFIX+'/map', '/map/ray_grid': PREFIX+'/ray_map'})
        # 生产节点工厂显式默认系统时钟；在独立仿真域通过公开参数服务切换，
        # 不修改生产入口的默认时钟策略，也不替换 node._clock。
        clock_clients = [self.create_client(SetParameters, '/'+name+'/set_parameters')
                         for name in ('base_provider', 'map_origin', 'map_exporter')]
        self.wait_for(lambda: all(client.service_is_ready() for client in clock_clients), '时钟参数服务')
        for client in clock_clients:
            request = SetParameters.Request()
            request.parameters = [Parameter('use_sim_time', value=True).to_parameter_msg()]
            response = self.call(client, request)
            assert all(result.successful for result in response.results), '仿真时钟切换失败'
        self.wait_for(lambda: self.odom_pub.get_subscription_count() >= 2 and self.cloud_pub.get_subscription_count() >= 1
                      and self.clock_pub.get_subscription_count() >= 4 and self.start_pub.get_subscription_count() >= 1,
                      '生产节点 DDS 发现')
        self.advance(0.1)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'odom', 'camera_init'
        tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        self.wait_for(lambda: self.save_client.service_is_ready() and self.floor_client.service_is_ready(), '建图服务')

    def messages(self, index, stamp):
        xyz, rpy = truth(index, self.scans)
        R = rotation(rpy)
        body_position = xyz+R@MOUNT_XYZ
        body_rotation = R@rotation(MOUNT_RPY)
        q = multiply_quaternion(quaternion(rpy), quaternion(MOUNT_RPY))
        msg = Odometry()
        set_stamp(msg.header.stamp, stamp)
        msg.header.frame_id, msg.child_frame_id = 'camera_init', 'body'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, body_position)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = map(float, q)
        origin = body_position+body_rotation@self.lidar_offset
        points = room_scan(origin, rpy[2], self.rng, self.noise, person=5 <= index < 35)
        cloud = PointCloud2()
        cloud.header = copy.deepcopy(msg.header)
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [PointField(name=axis, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, axis in enumerate('xyz')]
        cloud.point_step, cloud.row_step = 12, len(points)*12
        cloud.is_dense = True
        cloud.data = np.asarray(points, dtype='<f4').tobytes()
        self.expected[stamp_key(msg.header.stamp)] = (xyz, rpy)
        return msg, cloud

    def send_scan(self, index):
        self.advance(0.1)
        odom, cloud = self.messages(index, self.now)
        # 交替顺序，覆盖点云先到、位姿后到时的待匹配队列。
        if index % 2:
            self.odom_pub.publish(odom)
            self.pump(0.025)
            self.cloud_pub.publish(cloud)
            self.pump(0.025)
        else:
            self.cloud_pub.publish(cloud)
            self.pump(0.025)
            self.odom_pub.publish(odom)
            self.pump(0.025)
        return odom, cloud

    def call(self, client, request):
        future = client.call_async(request)
        self.wait_for(future.done, '服务响应')
        return future.result()

    def run(self):
        self.start()
        self.send_scan(0)
        self.advance(0.22)
        self.wait_for(lambda: any(s.get('state') == '需要确认地面高度' for s in self.statuses), '未确认地面保护')
        assert self.source_map is None, '未确认地面就发布了地图'
        assert not self.call(self.save_client, Trigger.Request()).success, '空地图竟然保存成功'
        request = SetParameters.Request()
        request.parameters = [Parameter('floor_confirmed', value=True).to_parameter_msg()]
        response = self.call(self.floor_client, request)
        assert all(result.successful for result in response.results), '地面参数设置失败'
        odom, cloud = self.send_scan(1)
        self.wait_for(lambda: stamp_key(odom.header.stamp) in self.raw_poses, '首条底盘中心位姿')
        start = Header()
        start.stamp = copy.deepcopy(odom.header.stamp)
        self.origin_xyz, self.origin_rpy = truth(1, self.scans)
        # 故意延迟送达起点信号，验证按消息时间而非收到时最新位置冻结。
        for index in range(2, 12):
            odom, cloud = self.send_scan(index)
        self.start_pub.publish(start)
        for index in range(12, self.scans):
            odom, cloud = self.send_scan(index)
        self.advance(0.22)
        final_stamp = stamp_key(cloud.header.stamp)
        self.wait_for(lambda: self.source_map is not None and self.final_map is not None
                      and stamp_key(self.source_map.header.stamp) >= final_stamp
                      and stamp_key(self.final_map.header.stamp) >= final_stamp, '最终源图与比赛坐标地图')
        self.wait_for(lambda: final_stamp in self.map_poses, '最终比赛坐标位姿')
        guards = self.fault_checks(odom, cloud)
        # 故障检查包含合法帧恢复。等其导出、保存后再计分，保证 PNG、
        # PGM/YAML 和 JSON 对应同一张最终地图，退出保存也不会改变它。
        recovery_stamp = stamp_key(self.source_map.header.stamp)
        self.wait_for(lambda: self.final_map is not None and stamp_key(self.final_map.header.stamp) == recovery_stamp,
                      '恢复帧的比赛地图导出')
        saved = self.call(self.save_client, Trigger.Request())
        assert saved.success, saved.message
        metrics = self.evaluate()
        metrics['guards'] = guards
        metrics['passed'] = all(metrics['checks'].values()) and all(guards.values())
        return metrics

    def fault_checks(self, odom, cloud):
        before_map = bytes(np.array(self.source_map.data, dtype=np.int8))
        checks = {'floor_confirmation_blocks_map': True, 'empty_map_save_rejected': True}
        # 每次等到可观测的拒绝计数更新，不能用“没崩溃”充当保护通过。
        for label in ('duplicate_stamp', 'wrong_frame', 'stale_stamp', 'future_stamp'):
            previous = max((s.get('rejected', 0) for s in self.statuses), default=0)
            bad = copy.deepcopy(cloud)
            if label == 'wrong_frame': bad.header.frame_id = 'map'
            elif label == 'stale_stamp': set_stamp(bad.header.stamp, self.now-2.)
            elif label == 'future_stamp': set_stamp(bad.header.stamp, self.now+2.)
            self.cloud_pub.publish(bad)
            self.advance(0.22)
            self.wait_for(lambda: any(s.get('rejected', 0) > previous for s in self.statuses), label+' 拒绝状态')
            checks[label+'_rejected'] = before_map == bytes(np.array(self.source_map.data, dtype=np.int8))
        # 新点云只有错 child 的位姿，不能误用上一帧的“最新位置”。
        self.advance(0.1)
        bad_odom, unmatched = self.messages(self.scans-1, self.now)
        bad_odom.child_frame_id = 'base_link'
        rejected_before = max(s.get('rejected', 0) for s in self.statuses)
        status_start = len(self.statuses)
        self.odom_pub.publish(bad_odom)
        self.cloud_pub.publish(unmatched)
        self.advance(0.22)
        checks['unmatched_pose_waits'] = any('等待与点云时间相差' in s.get('state', '') for s in self.statuses[status_start:])
        self.advance(0.5)
        # 等待状态不带 rejected 字段；等队列过期后读取累计数，分别覆盖
        # 错 child 位姿与无法匹配的过期点云这两次拒绝。
        checks['wrong_pose_child_rejected'] = max(s.get('rejected', 0) for s in self.statuses) >= rejected_before+2
        checks['unmatched_pose_does_not_change_map'] = before_map == bytes(np.array(self.source_map.data, dtype=np.int8))
        stale_from = len(self.statuses)
        self.advance(0.6)
        checks['stopped_input_reported'] = any('点云已停止或过期' in s.get('state', '') for s in self.statuses[stale_from:])
        poses_before = len(self.raw_poses)
        self.advance(0.3)
        checks['stale_base_not_republished'] = len(self.raw_poses) == poses_before
        # 后续合法帧必须可以恢复，坏输入不能让管线永久卡住。
        good, _ = self.send_scan(self.scans-1)
        recovery_stamp = stamp_key(good.header.stamp)
        self.advance(0.22)
        self.wait_for(lambda: stamp_key(self.source_map.header.stamp) == recovery_stamp
                      and recovery_stamp in self.map_poses, '故障后的有效输入恢复')
        checks['valid_input_recovers'] = True
        return checks

    def evaluate(self):
        source, final = self.source_map, self.final_map
        grid = np.asarray(source.data).reshape(source.info.height, source.info.width)
        def at(x, y):
            i, j = np.floor((np.array([x, y])-GRID_ORIGIN)/RESOLUTION).astype(int)
            return grid[j, i]
        wall_hits = []
        for axis, wall, lo, hi in ((0, -3., -2., 3.), (0, 4., -2., 3.), (1, -2., -3., 4.), (1, 3., -3., 4.)):
            for v in np.arange(lo+0.1, hi-0.05, 0.05):
                xy = [wall, v] if axis == 0 else [v, wall]
                i, j = np.floor((np.array(xy)-GRID_ORIGIN)/RESOLUTION).astype(int)
                wall_hits.append(bool(np.any(grid[j-1:j+2, i-1:i+2] == 100)))
        interior = [at(x, y) for x in np.arange(-2.8, 3.8, .1) for y in np.arange(-1.8, 2.8, .1)]
        outside = [at(x, y) for x in np.arange(-3.8, 5.8, .1) for y in np.arange(-2.8, 4.8, .1)
                   if x < -3.15 or x > 4.15 or y < -2.15 or y > 3.15]
        ghosts = [at(x, y) for x in np.arange(-.8, -.49, .025) for y in np.arange(.3, .61, .025)]
        occupied_y, occupied_x = np.where(grid == 100)
        occupied_xy = GRID_ORIGIN+(np.column_stack([occupied_x, occupied_y])+.5)*RESOLUTION
        wall_distance = np.min(np.column_stack([np.abs(occupied_xy[:, 0]+3), np.abs(occupied_xy[:, 0]-4),
                                               np.abs(occupied_xy[:, 1]+2), np.abs(occupied_xy[:, 1]-3)]), axis=1)
        raw_errors, map_errors, yaw_errors, velocity_errors = [], [], [], []
        Rmap = rotation([0., 0., -self.origin_rpy[2]])
        ordered = sorted(self.expected)
        previous = {key: ordered[i-1] for i, key in enumerate(ordered) if i}
        for key, message in sorted(self.map_poses.items()):
            if key not in self.expected: continue
            xyz, rpy = self.expected[key]
            target = Rmap@(xyz-self.origin_xyz)
            actual = message.pose.pose.position
            estimate = np.array([actual.x, actual.y, actual.z])
            map_error = float(np.linalg.norm(estimate-target))
            map_errors.append(map_error)
            q = message.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            yaw_errors.append(abs(math.atan2(math.sin(yaw-rpy[2]+self.origin_rpy[2]), math.cos(yaw-rpy[2]+self.origin_rpy[2]))))
            raw = self.raw_poses.get(key)
            if raw:
                p = raw.pose.pose.position
                raw_errors.append(float(np.linalg.norm(np.array([p.x, p.y, p.z])-xyz)))
            if key in previous and message.twist.covariance[0] < 1e5:
                old_key = previous[key]
                world_v = (xyz-self.expected[old_key][0])/((key-old_key)*1e-9)
                expected_body_v = rotation(rpy).T@world_v
                v = message.twist.twist.linear
                velocity_errors.append(float(np.linalg.norm(np.array([v.x, v.y, v.z])-expected_body_v)))
            self.pose_rows.append([key*1e-9, *target, *estimate, map_error])
        if not map_errors or not raw_errors or not velocity_errors:
            raise RuntimeError('没有足够的真实订阅输出用于评估')
        expected_origin = Rmap[:2, :2]@(GRID_ORIGIN-self.origin_xyz[:2])
        final_origin = final.info.origin.position
        q = final.info.origin.orientation
        final_yaw = 2*math.atan2(q.z, q.w)
        metadata_error = float(np.linalg.norm(np.array([final_origin.x, final_origin.y])-expected_origin))
        metadata_yaw_error = abs(math.atan2(math.sin(final_yaw+self.origin_rpy[2]), math.cos(final_yaw+self.origin_rpy[2])))
        disk, _ = read_pgm(str(self.output/'source/47_slam_map.pgm'))
        expected_pixels = np.where(grid == 100, 0, np.where(grid == 0, 254, 205)).astype(np.uint8)[::-1]
        final_disk, _ = read_pgm(str(self.output/'competition/47_map.pgm'))
        meta = yaml.safe_load((self.output/'competition/47_map.yaml').read_text())
        values = {
            'wall_coverage_within_one_cell': float(np.mean(wall_hits)),
            'interior_free_fraction': float(np.mean(np.array(interior) == 0)),
            'outside_unknown_fraction': float(np.mean(np.array(outside) == -1)),
            'person_ghost_occupied_samples': int(np.count_nonzero(np.array(ghosts) == 100)),
            'occupied_wall_distance_p95_m': float(np.percentile(wall_distance, 95)),
            'base_conversion_max_error_m': max(raw_errors), 'match_base_max_error_m': max(map_errors),
            'match_yaw_max_error_rad': max(yaw_errors), 'base_velocity_max_error_mps': max(velocity_errors),
            'map_origin_error_m': metadata_error, 'map_origin_yaw_error_rad': metadata_yaw_error,
            'input_scans': self.scans, 'received_base_samples': len(raw_errors),
            'received_match_samples': len(map_errors),
        }
        checks = {
            'wall_coverage': values['wall_coverage_within_one_cell'] > .97,
            'interior_free': values['interior_free_fraction'] > .97,
            'outside_unknown': values['outside_unknown_fraction'] == 1.,
            'temporary_person_cleared': values['person_ghost_occupied_samples'] == 0,
            'wall_thickness': values['occupied_wall_distance_p95_m'] < .08,
            'base_mount_conversion': max(raw_errors) < 1e-6,
            'match_start_and_trajectory': max(map_errors) < 1e-6 and max(yaw_errors) < 1e-6,
            'velocity_conversion': max(velocity_errors) < 1e-6,
            'received_enough_samples': len(map_errors) >= .85*self.scans,
            'map_transform': metadata_error < 1e-6 and metadata_yaw_error < 1e-6 and final.header.frame_id == 'map',
            'source_map_pgm_orientation': np.array_equal(disk, expected_pixels),
            'exported_map_cells_preserved': list(source.data) == list(final.data) and np.array_equal(disk, final_disk),
            'exported_yaml_origin': np.allclose(meta['origin'], [*expected_origin, -self.origin_rpy[2]], atol=1e-6, rtol=0),
        }
        with (self.output/'trajectory.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'truth_x', 'truth_y', 'truth_z', 'estimated_x', 'estimated_y', 'estimated_z', 'error_m'])
            writer.writerows(self.pose_rows)
        self.preview(disk, checks)
        return {'metrics': values, 'checks': checks}

    def preview(self, disk, checks):
        scale = 5
        image = Image.fromarray(disk).convert('RGB').resize((disk.shape[1]*scale, disk.shape[0]*scale), Image.NEAREST)
        draw = ImageDraw.Draw(image)
        def pixel(xy):
            ij = (np.array(xy)-GRID_ORIGIN)/RESOLUTION
            return (float(ij[0]*scale), float((disk.shape[0]-ij[1])*scale))
        for a, b in (((-3, -2), (4, -2)), ((4, -2), (4, 3)), ((4, 3), (-3, 3)), ((-3, 3), (-3, -2))):
            draw.line([pixel(a), pixel(b)], fill=(30, 120, 240), width=2)
        truth_xy = [truth(k, self.scans)[0][:2] for k in range(self.scans)]
        draw.line([pixel(p) for p in truth_xy], fill=(30, 160, 70), width=4)
        R = rotation([0., 0., self.origin_rpy[2]])[:2, :2]
        for row in self.pose_rows[::4]:
            xy = R@np.array(row[4:6])+self.origin_xyz[:2]
            x, y = pixel(xy)
            draw.ellipse((x-3, y-3, x+3, y+3), fill=(230, 60, 50))
        draw.rectangle((8, 8, 730, 76), fill='white')
        draw.text((16, 14), 'SYNTHETIC REGISTERED CLOUD / real ROS 2 nodes / Point-LIO BYPASSED', fill='black')
        draw.text((16, 34), 'Blue: true walls   Green: true base path   Red: received base path', fill='black')
        draw.text((16, 54), f'Range noise: {self.noise:.3f} m   Geometry checks: {sum(checks.values())}/{len(checks)}', fill='black')
        image.save(self.output/'overview.png')

    def close(self):
        # 只终止此仿真拥有的进程组，不扫其他 ROS 进程。
        for _, proc in reversed(self.processes):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass  # 退出检查和发信号之间子进程可能已自行结束。
        for _, proc in reversed(self.processes):
            try:
                proc.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3.)
        for log in self.logs: log.close()
        self.destroy_node()
        return {name: proc.returncode for name, proc in self.processes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--scans', type=int, default=160)
    parser.add_argument('--seed', type=int, default=47)
    parser.add_argument('--scenario', choices=['all', 'clean', 'range_noise'], default='all')
    parser.add_argument('--inject-mount-error', action='store_true', help='故意将底盘安装偏移写错 25cm；验收必须失败')
    args = parser.parse_args()
    if args.scans < 120: parser.error('--scans 至少 120，确保临时行人离开后仍有足够重观测')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('请设置 ROS_LOCALHOST_ONLY=1 和独立 ROS_DOMAIN_ID（例如 87）')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    results = []
    report = {'kind': 'registered_cloud_ros2_integration_simulation', 'seed': args.seed,
              'Point_LIO_executed': False, 'hardware_accessed': False,
              'production_nodes_per_scenario': 4, 'scenarios': results,
              'limitations': ['合成已配准点云与已知位姿；没有估计原始雷达/IMU定位',
                              '不复现MID360扫描模式、运动畸变、IMU噪声或漂移',
                              '浮点级位姿误差只证明坐标转换；不能代表实机定位精度',
                              '没有运行目标跟踪、九列评分提交或NUC实时负载验证']}
    scenarios = [('clean', 0.), ('range_noise', .01)] if args.scenario == 'all' else [(args.scenario, 0. if args.scenario == 'clean' else .01)]
    for name, noise in scenarios:
        folder = output/name
        folder.mkdir()
        # 本节点的静态 TF 也隔离；它不使用仿真时钟做输入年龄判断。
        rclpy.init(args=['--ros-args', '-r', '/tf_static:='+PREFIX+'/tf_static'])
        app = Simulation(folder, noise, args.scans, args.seed, args.inject_mount_error)
        try:
            result = {'name': name, **app.run()}
        except Exception as exc:
            result = {'name': name, 'passed': False, 'error': repr(exc)}
            (folder/'failure.log').write_text(traceback.format_exc())
        finally:
            exit_codes = app.close()
            rclpy.shutdown()
        result['node_exit_codes'] = exit_codes
        result['passed'] = result['passed'] and all(code == 0 for code in exit_codes.values())
        results.append(result)
        (folder/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        report['passed'] = all(row['passed'] for row in results)
        (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
