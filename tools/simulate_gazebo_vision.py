#!/usr/bin/env python3
"""Gazebo 真正渲染 ArUco → 生产 PnP/TF/目标估计，并注入观测传输故障。

复用三维雷达、IMU、Point-LIO 和建图运行器；相机固定在移动平台上。
不把目标坐标或 Gazebo 平台真值发给生产估计节点。
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
import traceback
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image as PilImage, ImageDraw
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge
from team43_interfaces.msg import MarkerObservation, TargetState
from team43_common import frames
from team43_common.qos_utils import sensor_qos
from simulate_gazebo_lio import GazeboLio, NS, ROOT, ORIGIN, RESOLUTION, seconds, envelope, point_lio_image_build, audit_received_inputs


MARKER = np.array([2.8, 0., .9])
MARKER_SIZE = .1
MARKER_CENTER = MARKER+np.array([-.0042, 0., 0.])
BODY_IN_BASE = np.array([.08, -.03, .06])
CAMERA_IN_BODY = np.array([.15, -.08, .12])
CAMERA_RPY = np.array([.02, -.04, .06])
WIDTH, HEIGHT, HFOV = 800, 600, .65
# 故障发生在 PnP 与估计器之间，时间单位为仿真秒；保留图像原始 stamp。
FAULTS = {'delay': (8., 11.), 'drop': (16., 19.), 'replay': (24., 27.)}


def add(parent, name, text):
    item = ET.SubElement(parent, name)
    item.text = str(text)
    return item


def box(link, name, xyz, size, color):
    visual = ET.SubElement(link, 'visual', name=name)
    add(visual, 'pose', ' '.join(map(str, xyz))+' 0 0 0')
    add(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
    material = ET.SubElement(visual, 'material')
    for key in ('ambient', 'diffuse', 'emissive'):
        add(material, key, color)
    add(visual, 'cast_shadows', 'false')


class GazeboVision(GazeboLio):
    def __init__(self, output, timeout, factor, record_bag, scan_lines, scene, sigma_floor):
        super().__init__(output, timeout, factor, record_bag, scan_lines)
        self.scene = scene
        self.sigma_floor = sigma_floor
        self.images, self.infos, self.observations, self.states = [], [], [], []
        self.deliveries, self.pending = [], []
        self.fault_counts = {key: 0 for key in FAULTS}
        self.replay_message = None
        self.bridge_cv = CvBridge()
        self.pub_observation = self.create_publisher(MarkerObservation, NS+'/vision/input', sensor_qos(20))
        self.create_subscription(Image, NS+'/camera/image', self.on_image, 5)
        self.create_subscription(CameraInfo, NS+'/camera/camera_info', self.on_info, 5)
        self.create_subscription(MarkerObservation, NS+'/vision/pnp', self.on_observation, sensor_qos(100))
        self.create_subscription(TargetState, NS+'/vision/state', self.on_state, sensor_qos(500))
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_timer(.01, self.deliver_pending)
        self.prepare_world()

    def prepare_world(self):
        tree = ET.parse(self.output/'world.sdf')
        world = tree.find('world')
        if self.scene == 'features':
            # 加入不同方位的近距离结构，和仅两箱体的稀疏房间作对照。
            for i, (xyz, size) in enumerate((
                ([1.4, -.8, .6], [.4, .3, 1.2]), ([1.8, .7, .9], [.3, .4, 1.8]),
                ([-.7, -1.2, .6], [.6, .4, 1.2]), ([-1.1, 1.3, 1.], [.5, .5, 2.]),
                ([.8, 2.1, .7], [.5, .7, 1.4]))):
                feature = ET.SubElement(world, 'model', name=f'feature_{i}')
                add(feature, 'static', 'true'); add(feature, 'pose', ' '.join(map(str, xyz))+' 0 0 0')
                box(ET.SubElement(feature, 'link', name='shape'), 'visual', [0, 0, 0], size, '.5 .5 .5 1')
        body = world.find("model[@name='lidar_rig']/link[@name='body']")
        sensor = ET.SubElement(body, 'sensor', name='camera', type='camera')
        add(sensor, 'pose', ' '.join(map(str, [*CAMERA_IN_BODY, *CAMERA_RPY])))
        add(sensor, 'always_on', 'true')
        add(sensor, 'update_rate', 15)
        add(sensor, 'topic', NS+'/camera/image')
        camera = ET.SubElement(sensor, 'camera')
        add(camera, 'horizontal_fov', HFOV)
        add(camera, 'camera_info_topic', NS+'/camera/camera_info')
        image = ET.SubElement(camera, 'image')
        for key, value in (('width', WIDTH), ('height', HEIGHT), ('format', 'R8G8B8')):
            add(image, key, value)
        clip = ET.SubElement(camera, 'clip')
        add(clip, 'near', .05); add(clip, 'far', 15.)
        # 用真实三维黑白格构成码，免去外部纹理下载；未投影或合成输入图像。
        marker = ET.SubElement(world, 'model', name='stationary_aruco')
        add(marker, 'static', 'true')
        add(marker, 'pose', ' '.join(map(str, MARKER))+' 0 0 0')
        link = ET.SubElement(marker, 'link', name='board')
        box(link, 'white_border', [0, 0, 0], [.008, .14, .14], '1 1 1 1')
        bits = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 6)
        for row, col in zip(*np.where(bits == 0)):
            box(link, f'cell_{row}_{col}', [-.0041, (.5-(col+.5)/6)*MARKER_SIZE, (.5-(row+.5)/6)*MARKER_SIZE],
                [.0002, MARKER_SIZE/6+.00001, MARKER_SIZE/6+.00001], '0 0 0 1')
        tree.write(self.output/'world.sdf', encoding='unicode', xml_declaration=True)
        fx = WIDTH/(2*math.tan(HFOV/2))
        # YAML 为节点启动必需的备用内参，运行时由真实 CameraInfo 覆盖。
        import yaml
        (self.output/'camera.yaml').write_text(yaml.safe_dump({'camera': {
            'width': WIDTH, 'height': HEIGHT, 'fx': fx, 'fy': fx,
            'cx': WIDTH/2, 'cy': HEIGHT/2, 'distortion': [0.]*5}}))

    def launch(self, name, command, env=None):
        if name == 'bridge':
            i = command.index('--ros-args')
            command[i:i] = [NS+'/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            NS+'/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo']
        elif name == 'bag':
            command += [NS+'/camera/image', NS+'/camera/camera_info', NS+'/vision/pnp',
                        NS+'/vision/input', NS+'/vision/state', NS+'/tf', NS+'/tf_static']
        super().launch(name, command, env)

    def start(self):
        self.launch_ros('vision_detector', 'team43_perception', 'aruco_detector_node', {
            'perception.image_topic': NS+'/camera/image',
            'perception.marker_corners_topic': NS+'/vision/corners',
            'perception.detect_rate_hz': 20.,
        }, {})
        self.launch_ros('vision_pnp', 'team43_perception', 'marker_pnp_node', {
            'perception.marker_corners_topic': NS+'/vision/corners',
            'perception.camera_info_topic': NS+'/camera/camera_info',
            'perception.camera_info_source': 'topic',
            'config.virtual_yaml': str(self.output/'camera.yaml'),
            'perception.marker_size_m': MARKER_SIZE,
        }, {'/perception/marker_cam': NS+'/vision/pnp'})
        self.launch_ros('vision_base', 'team43_localization', 'base_pose_provider_node', {
            'localization.base_pose_source': 'odom',
            'localization.odom_topic': NS+'/lio/aft_mapped_to_init',
            'localization.odom_frame_id': 'camera_init', 'localization.odom_child_frame_id': 'body',
            'localization.sensor_in_base_xyz': BODY_IN_BASE.tolist(),
            'localization.sensor_extrinsics_confirmed': True,
            'localization.parent_transform_confirmed': True,
            'localization.odom_twist_source': 'pose_difference',
        }, {'/gimbal/base_pose_raw': NS+'/vision/base_raw', '/judge/replay_start': NS+'/unused_start'})
        self.launch_ros('vision_origin', 'team43_localization', 'map_frame_broadcaster_node', {
            'localization.origin_mode': 'base_init', 'localization.origin_settle_s': .5,
        }, {'/gimbal/base_pose_raw': NS+'/vision/base_raw', '/gimbal/base_pose': NS+'/vision/base',
            '/gimbal/t_map_world': NS+'/vision/t_map_world'})
        self.launch_ros('vision_estimator', 'team43_estimation', 'target_estimator_node', {
            'estimation.geometry_confirmed': True,
            # 这里只配置仿真：组合观测包含厘米级定位误差，不能只按毫米级
            # 像素横向噪声估计 R。实际外参和定位协方差仍需实测标定。
            'perception.r_min_sigma': self.sigma_floor,
        }, {'/perception/marker_cam': NS+'/vision/input', '/estimation/target_state': NS+'/vision/state',
            '/task1/gimbal_reset_epoch': NS+'/vision/epoch'})
        # 节点工厂固定实车默认墙钟，仅在本次仿真通过公开服务切换时钟。
        for name in ('vision_pnp', 'vision_base', 'vision_origin'):
            client = self.create_client(SetParameters, '/'+name+'/set_parameters')
            if not client.wait_for_service(timeout_sec=15.): raise RuntimeError(name+' 参数服务未就绪')
            request = SetParameters.Request()
            request.parameters = [Parameter('use_sim_time', value=True).to_parameter_msg()]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
            if not future.done() or not all(r.successful for r in future.result().results):
                raise RuntimeError(name+' 仿真时钟未设置成功')
        # 固定相机安装，非零平移及三轴旋转；base↔IMU 的杠杆臂也非零。
        transforms = []
        for parent, child, xyz, R in (
            ('base_link', 'camera_link', BODY_IN_BASE+CAMERA_IN_BODY, frames.rpy_to_mat(CAMERA_RPY)),
            ('camera_link', 'camera_optical_frame', np.zeros(3), np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]))):
            msg = TransformStamped(); msg.header.frame_id = parent; msg.child_frame_id = child
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, xyz)
            q = frames.quat_from_mat(R)
            msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z = map(float, q)
            transforms.append(msg)
        self.static_tf.sendTransform(transforms)
        super().start()

    def on_image(self, msg):
        t = seconds(msg.header.stamp)
        self.images.append(t)
        # 保留原始 Gazebo 帧，便于独立目视核查；不参与检测输入生成。
        if len(self.images) == 15 or len(self.images) % 150 == 0:
            image = self.bridge_cv.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            PilImage.fromarray(image).save(self.output/f'camera_{len(self.images):05d}.png')

    def on_info(self, msg):
        if not self.infos:
            self.infos.append({'width': msg.width, 'height': msg.height, 'k': list(msg.k), 'd': list(msg.d)})

    def on_state(self, msg):
        self.states.append([seconds(msg.header.stamp), msg.cur_x, msg.cur_y, int(msg.valid),
                            msg.track_status, msg.last_obs_age, msg.pred_x, msg.pred_y])

    def relative(self, now):
        return now-self.motion_start if self.motion_start is not None else -100.

    def on_observation(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        self.observations.append([seconds(msg.header.stamp), now, int(msg.valid), msg.x, msg.y, msg.z, msg.reproj_err_px])
        t = self.relative(now)
        if FAULTS['drop'][0] <= t < FAULTS['drop'][1]:
            self.fault_counts['drop'] += 1
            return
        if FAULTS['delay'][0] <= t < FAULTS['delay'][1]:
            self.pending.append((now+.45, copy.deepcopy(msg)))
            self.fault_counts['delay'] += 1
            return
        if FAULTS['replay'][0] <= t < FAULTS['replay'][1]:
            if self.replay_message is not None:
                self.deliver(self.replay_message, now, 'replay')
                self.fault_counts['replay'] += 1
            return
        if msg.valid: self.replay_message = copy.deepcopy(msg)
        self.deliver(msg, now, 'normal')

    def deliver(self, msg, now, mode):
        self.pub_observation.publish(msg)
        self.deliveries.append([now, seconds(msg.header.stamp), int(msg.valid), mode])

    def deliver_pending(self):
        now = self.get_clock().now().nanoseconds*1e-9
        remaining = []
        for deadline, msg in self.pending:
            if now >= deadline: self.deliver(msg, now, 'delay')
            else: remaining.append((deadline, msg))
        self.pending = remaining

    def drive(self, now):
        cmd = Twist()
        elapsed = self.relative(now)
        if 0 <= elapsed < 32:
            leg, t = int(elapsed//4), elapsed % 4
            gain = envelope(t)
            if leg in (0, 1): cmd.linear.x = (.24 if leg == 0 else -.24)*gain
            elif leg in (2, 3): cmd.linear.y = (.15 if leg == 2 else -.15)*gain
            elif leg in (4, 5): cmd.angular.z = (.06 if leg == 4 else -.06)*gain
            elif leg in (6, 7): cmd.linear.x = (.15 if leg == 6 else -.15)*gain
        self.cmd_pub.publish(cmd)

    def evaluate(self, elapsed_wall):
        report = super().evaluate(elapsed_wall)
        states, obs = np.array(self.states), np.array(self.observations)
        if len(states) < 100 or len(obs) < 50 or not self.infos:
            raise RuntimeError('图像、CameraInfo、PnP 或目标状态样本不足')
        expected = MARKER_CENTER[:2]+BODY_IN_BASE[:2]
        t = states[:, 0]-self.motion_start
        normal = ((t > 1) & (t < 8)) | ((t > 12) & (t < 16)) | ((t > 20) & (t < 24)) | ((t > 28) & (t < 35))
        valid = normal & (states[:, 3] == 1)
        if np.sum(valid) < 50: raise RuntimeError('正常窗口缺少足够有效目标状态')
        errors = np.linalg.norm(states[valid, 1:3]-expected, axis=1)
        pred_errors = np.linalg.norm(states[valid, 6:8]-expected, axis=1)
        # 独立审核渲染/PnP/安装模型，只离线评分，不提供给估计器。
        gt = np.asarray(self.truth)
        optical_R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        camera_R = frames.rpy_to_mat(CAMERA_RPY)@optical_R
        pnp_errors = []
        for stamp, _, valid_msg, x, y, z, _ in self.observations:
            if not valid_msg or not gt[0, 0] <= stamp <= gt[-1, 0]: continue
            yaw = np.interp(stamp, gt[:, 0], np.unwrap(gt[:, 4]))
            rotation = frames.rotz(yaw)
            position = np.array([np.interp(stamp, gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
            measured = position+rotation@(CAMERA_IN_BODY+camera_R@np.array([x, y, z]))
            pnp_errors.append(np.linalg.norm(measured-MARKER_CENTER))
        vision_checks = {
            'real_images_and_intrinsics': len(self.images) > 300 and self.infos[0]['width'] == WIDTH,
            'actual_pnp_detections': int(np.sum(obs[:, 2])) > 300,
            'image_stamp_preserved': all(any(abs(stamp-image_t) < 1e-8 for image_t in self.images) for stamp in obs[obs[:, 2] == 1, 0]),
            'normal_valid_fraction': float(np.mean(states[normal, 3])) > .9,
            'target_world_rmse': float(np.sqrt(np.mean(errors**2))) < .15,
            'target_world_max': float(np.max(errors)) < .35,
            'stationary_target_prediction_rmse': float(np.sqrt(np.mean(pred_errors**2))) < .2,
            'rendered_pnp_geometry': bool(pnp_errors) and float(np.sqrt(np.mean(np.square(pnp_errors)))) < .05,
        }
        fault_metrics = {}
        for name, (start, end) in FAULTS.items():
            fault = (t > start+1.) & (t < end-.1)
            recovery = (t > end+1.) & (t < end+2.5)
            if not np.any(fault) or not np.any(recovery): raise RuntimeError(name+' 故障或恢复窗口无采样')
            invalid_fraction = float(np.mean(states[fault, 3] == 0))
            recovered_fraction = float(np.mean(states[recovery, 3] == 1))
            fault_metrics[name] = {'injected_messages': self.fault_counts[name],
                                   'invalid_fraction': invalid_fraction, 'recovery_valid_fraction': recovered_fraction}
            vision_checks[name+'_invalidates'] = invalid_fraction > .95 and self.fault_counts[name] > 20
            vision_checks[name+'_recovers'] = recovered_fraction > .9
        delayed = [arrival-stamp for arrival, stamp, valid_msg, mode in self.deliveries if mode == 'delay' and valid_msg]
        vision_checks['delayed_stamps_exceed_deadline'] = bool(delayed) and min(delayed) >= .45-1e-6
        replayed = [stamp for _, stamp, valid_msg, mode in self.deliveries if mode == 'replay' and valid_msg]
        vision_checks['replay_keeps_original_stamp'] = len(replayed) > 20 and max(replayed) == min(replayed)
        if self.scene == 'features':
            # 新障碍会挡住远处墙面；按独立 SDF 几何及真值轨迹计算预期
            # 可见墙面，保留原始全墙覆盖率，不能把合理的未知区当漏建。
            visible_coverage, visible_fraction, visible_count = self.visible_wall_coverage()
            report['metrics'].update(visible_wall_coverage_within_0_1m=visible_coverage,
                                     wall_visibility_fraction=visible_fraction,
                                     visible_wall_samples=visible_count)
            report['checks']['wall_coverage'] = visible_count > 100 and visible_coverage > .85
        # 同一定位、地图门槛；本场景往返运动的预设跨度为0.84m，仍满足原>0.8m。
        report['vision_metrics'] = {
            'images': len(self.images), 'pnp_valid': int(np.sum(obs[:, 2])),
            'camera_info': self.infos[0], 'expected_target_map_xy': expected.tolist(),
            'target_position_rmse_m': float(np.sqrt(np.mean(errors**2))),
            'target_position_max_m': float(np.max(errors)),
            'target_prediction_rmse_m': float(np.sqrt(np.mean(pred_errors**2))),
            'pnp_world_geometry_rmse_m': float(np.sqrt(np.mean(np.square(pnp_errors)))),
            'normal_valid_fraction': float(np.mean(states[normal, 3])), 'faults': fault_metrics,
        }
        report['checks'].update(vision_checks)
        report['passed'] = all(report['checks'].values())
        self.vision_preview(states, expected)
        return report

    def write_traces(self):
        for name, header, rows in (
            ('target_states.csv', ['t', 'x', 'y', 'valid', 'status', 'age', 'pred_x', 'pred_y'], self.states),
            ('pnp_observations.csv', ['image_t', 'arrival_t', 'valid', 'x', 'y', 'z', 'reprojection_px'], self.observations),
            ('observation_delivery.csv', ['arrival_t', 'image_t', 'valid', 'mode'], self.deliveries)):
            with (self.output/name).open('w', newline='') as f:
                writer = csv.writer(f); writer.writerow(header); writer.writerows(rows)

    def visible_wall_coverage(self):
        gt = np.asarray(self.truth)[::20]
        c, s = np.cos(gt[:, 4]), np.sin(gt[:, 4])
        lx, ly, lz = self.lidar_offset
        origins = gt[:, 1:3]+np.column_stack([c*lx-s*ly, s*lx+c*ly])
        sensor_z = float(np.mean(gt[:, 3])+lz)
        obstacles = []
        for model in ET.parse(self.output/'world.sdf').findall('.//world/model'):
            if model.findtext('static') != 'true': continue
            visual = model.find('.//visual')
            size = visual.find('geometry/box/size') if visual is not None else None
            if size is None: continue
            center = np.array(list(map(float, model.findtext('pose').split())))[:3]
            center += np.array(list(map(float, visual.findtext('pose', '0 0 0 0 0 0').split())))[:3]
            half = np.array(list(map(float, size.text.split())))/2
            if center[2]-half[2] <= sensor_z <= center[2]+half[2]:
                obstacles.append((center[:2]-half[:2], center[:2]+half[:2]))
        msg = self.latest_map
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        covered, total = [], 0
        for axis, wall, lo, hi in ((0, -4., -3., 3.), (0, 4., -3., 3.), (1, -3., -4., 4.), (1, 3., -4., 4.)):
            for v in np.arange(lo+.15, hi-.1, .05):
                target = np.array([wall, v] if axis == 0 else [v, wall])
                direction = target-origins
                blocked = np.zeros(len(origins), dtype=bool)
                for lower, upper in obstacles:
                    parallel = np.abs(direction) < 1e-10
                    inverse = np.divide(1., direction, out=np.zeros_like(direction), where=~parallel)
                    a, b = (lower-origins)*inverse, (upper-origins)*inverse
                    near, far = np.minimum(a, b), np.maximum(a, b)
                    near[parallel], far[parallel] = -np.inf, np.inf
                    parallel_outside = np.any(parallel & ((origins < lower) | (origins > upper)), axis=1)
                    entry, leave = near.max(axis=1), far.min(axis=1)
                    blocked |= ~parallel_outside & (leave >= np.maximum(entry, 0.)) & (entry < 1.-1e-6) & (leave > 0.)
                total += 1
                if not np.any(~blocked): continue
                i, j = np.floor((target-ORIGIN)/RESOLUTION).astype(int)
                covered.append(bool(np.any(grid[j-2:j+3, i-2:i+3] == 100)))
        if not covered: raise RuntimeError('场景没有可评价的可见墙面')
        return float(np.mean(covered)), len(covered)/total, len(covered)

    def vision_preview(self, states, expected):
        image = PilImage.new('RGB', (1100, 620), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((25, 15), 'REAL GAZEBO RGB -> ARUCO -> PNP + POINT-LIO TF -> TARGET ESTIMATOR', fill='black')
        panels = [(60, 65, 1050, 290), (60, 355, 1050, 555)]
        max_t = 35.
        t = states[:, 0]-self.motion_start
        for panel in panels:
            left, top, right, bottom = panel
            for name, (a, b) in FAULTS.items():
                draw.rectangle((left+a/max_t*(right-left), top, left+b/max_t*(right-left), bottom), fill=(255, 235, 215))
                draw.text((left+a/max_t*(right-left)+3, top+3), name, fill='black')
            draw.rectangle(panel, outline='gray')
        left, top, right, bottom = panels[0]
        errors = np.linalg.norm(states[:, 1:3]-expected, axis=1)
        points = [(left+x/max_t*(right-left), bottom-min(error, .4)/.4*(bottom-top)) for x, error, v in zip(t, errors, states[:, 3]) if 0 <= x <= max_t and v]
        for x, y in points: draw.ellipse((x-1, y-1, x+1, y+1), fill=(30, 100, 210))
        draw.text((60, 43), 'Valid target position error (0 .. 0.4 m); orange = injected observation fault', fill='black')
        left, top, right, bottom = panels[1]
        draw.text((60, 330), 'Target valid flag (0 / 1): rejects stale data, resumes on fresh images', fill='black')
        points = [(left+x/max_t*(right-left), bottom-15-v*(bottom-top-30)) for x, v in zip(t, states[:, 3]) if 0 <= x <= max_t]
        draw.line(points, fill=(20, 140, 60), width=2)
        for tick in range(0, 36, 5): draw.text((left+tick/max_t*(right-left), 570), str(tick)+'s', fill='black')
        image.save(self.output/'vision_overview.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=600.)
    parser.add_argument('--real-time-factor', type=float, default=.5)
    parser.add_argument('--record-bag', action='store_true')
    parser.add_argument('--scan-lines', type=int, choices=[16, 32], default=16)
    parser.add_argument('--scene', choices=['room', 'features'], default='features',
                        help='两箱体房间，或增加不同方位近距离结构的对照场景')
    parser.add_argument('--observation-sigma-floor', type=float, default=.03,
                        help='仅估计节点的 map 观测噪声下限（米），正式配置不变')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID') or not os.environ.get('IGN_PARTITION'):
        parser.error('必须设置 ROS_LOCALHOST_ONLY=1、独立 ROS_DOMAIN_ID 和 IGN_PARTITION')
    if not 0 < args.real_time_factor <= 1 or not 30 <= args.timeout <= 3600:
        parser.error('real-time-factor 须在 (0,1]，timeout 须为30～3600秒')
    if not 0 < args.observation_sigma_floor <= 1:
        parser.error('observation-sigma-floor 须在 (0,1] 米')
    output = args.output_dir.resolve(); output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=['--ros-args', '-r', '/tf:='+NS+'/tf', '-r', '/tf_static:='+NS+'/tf_static'])
    app = GazeboVision(output, args.timeout, args.real_time_factor, args.record_bag,
                       args.scan_lines, args.scene, args.observation_sigma_floor)
    report = {'kind': 'gazebo_rgb_lidar_imu_target_estimation',
              'point_lio_image_build': point_lio_image_build(),
              'world_source': str(app.world_source), 'world_source_sha256': app.world_source_sha256,
              'generated_world_sha256': hashlib.sha256((output/'world.sdf').read_bytes()).hexdigest(),
              'source_archive_role': 'livox_messages_only',
              'ground_truth_used_as_estimator_input': False, 'hardware_accessed': False,
              'imu_model': 'imu_measurement_update', 'scan_lines': args.scan_lines, 'scene': args.scene,
              'observation_sigma_floor_m': args.observation_sigma_floor,
              'source_archive_sha256': hashlib.sha256((ROOT/'archives/livox_ros_driver2.tar.gz').read_bytes()).hexdigest(),
              'limitations': ['固定相机，未覆盖云台关节、视觉控制器或真实电机',
                              '针孔无畸变相机，0.1m静止ArUco；未覆盖运动目标预测精度',
                              '故障注入在PnP之后，未覆盖相机断流及进程重启',
                              '规则多线雷达和悬浮施力平台；未模拟MID360扫描或轮胎',
                              '门槛是仿真工程回归标准；没有运行180秒九列评分提交']}
    try: report.update(app.run())
    except Exception as exc:
        report.update(passed=False, error=repr(exc), images=len(app.images), states=len(app.states), pnp=len(app.observations))
        (output/'failure.log').write_text(traceback.format_exc())
    finally:
        try:
            app.write_traces()
        finally:
            report['node_exit_codes'] = app.close()
            if rclpy.ok(): rclpy.shutdown()
    report['clean_shutdown'] = all(code == 0 for code in report['node_exit_codes'].values())
    report['Gazebo_executed'] = bool(app.images) and bool(app.raw_scans) and bool(app.truth)
    report['Point_LIO_executed'] = bool(app.estimates)
    audit_received_inputs(report, output, app.published_lidar_stamps, [row[0] for row in app.imus])
    report['passed'] = report['passed'] and report['clean_shutdown']
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
