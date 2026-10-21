#!/usr/bin/env python3
"""同一仿真 bag 的 Point-LIO 对照回放，保留测量时间并独立对照真值。

只回放 clock、Livox 和 IMU，不回放旧定位、TF、图像或目标结果。
预编译的诊断二进制通过 --executable 指定；默认使用镜像安装版并记录构建清单。
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
import rosbag2_py
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import yaml


NS = '/sim/gazebo'
DRAIN_TOLERANCE_S = .001


def last_lidar_point_stamp(source, params):
    """以实际可预处理的末个点确定排空目标，不要求定位延伸到真值尾帧。"""
    if (params['common'].get('cut_frame', False) or params['common'].get('con_frame', False)
            or params['preprocess']['lidar_type'] != 1):
        raise ValueError('排空审核目前仅支持未切帧、未拼帧的 Livox 仿真记录')
    point_filter = int(params['point_filter_num'])
    if point_filter < 1: raise ValueError('point_filter_num 须为正整数')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source/'sensor_bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[NS+'/livox/lidar', NS+'/imu']))
    final_lidar, last_imu = None, None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == NS+'/livox/lidar':
            final_lidar = data
        else:
            stamp = deserialize_message(data, Imu).header.stamp
            last_imu = stamp.sec+stamp.nanosec*1e-9
    if final_lidar is None: raise ValueError('bag 没有 Livox 输入')
    msg = deserialize_message(final_lidar, CustomMsg)
    if msg.point_num != len(msg.points) or msg.point_num < 2:
        raise ValueError('末条 Livox 消息的点数无效')
    points = np.asarray([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
    # 与归档 avia_handler 一致：跳过第0点，通道/tag有效计数后按序抽样。
    eligible = np.asarray([i for i, p in enumerate(msg.points) if i > 0
                           and p.line < params['preprocess']['scan_line']
                           and (p.tag & 0x30) in (0x00, 0x10)], dtype=np.int64)
    eligible = eligible[point_filter-1::point_filter]
    if not len(eligible): raise ValueError('末条 Livox 消息没有有效通道点')
    full = np.zeros_like(points)
    full[eligible] = points[eligible]
    distance = np.sum(points[eligible]**2, axis=1)
    # 归档 AVIA 预处理器的 det_range 保持构造默认值1000m。
    usable = (np.isfinite(points[eligible]).all(axis=1)
              & (distance >= params['preprocess']['blind']**2)
              & (distance <= 1000.**2)
              & np.any(np.abs(points[eligible]-full[eligible-1]) > 1e-7, axis=1))
    eligible = eligible[usable]
    if not len(eligible): raise ValueError('末条 Livox 消息没有可预处理的点')
    # AVIA 将纳秒 offset 转为 float 毫秒，再在 sync_packages 中还原秒。
    offset_ms = max(float(np.float32(msg.points[i].offset_time)/np.float32(1e6)) for i in eligible)
    expected = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9+offset_ms*1e-3
    if params['mapping'].get('imu_en', True):
        if last_imu is None or last_imu < expected-1e-6:
            raise ValueError('bag 的末条 Livox 点没有足够晚的 IMU 支撑')
    return expected


class Replay(Node):
    def __init__(self, source, output, executable, overrides, variant, rate, qos_depth,
                 official_sensor_parameters=False):
        super().__init__('lio_diagnostic_replay', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.source, self.output, self.executable = source, output, executable
        self.rows, self.handles, self.processes = [], [], []
        self.rate, self.variant = rate, variant
        self.qos_depth = qos_depth
        self.official_sensor_parameters = official_sensor_parameters
        self.create_subscription(Odometry, NS+'/diagnosis/aft_mapped_to_init', self.observe, qos_profile_sensor_data)
        config = yaml.safe_load((source/'point_lio.yaml').read_text())
        params = config['/**']['ros__parameters']
        for key, value in overrides.items():
            obj = params
            parts = key.split('.')
            for part in parts[:-1]: obj = obj[part]
            if parts[-1] not in obj: raise ValueError('未知参数 '+key)
            obj[parts[-1]] = value
        if self.official_sensor_parameters:
            if qos_depth is not None: params['sensor_qos_depth'] = qos_depth
            params['input_audit_path'] = str(output/'input_diagnostics.csv')
        self.params = params
        (output/'point_lio.yaml').write_text(yaml.safe_dump(config))

    def observe(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.rows.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, p.x, p.y, p.z, yaw])

    def launch(self, name, command, env=None):
        handle = (self.output/(name+'.log')).open('w')
        self.handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=env)
        self.processes.append((name, process))
        return process

    def run(self):
        self.expected_last_stamp = last_lidar_point_stamp(self.source, self.params)
        metadata = yaml.safe_load((self.source/'sensor_bag/metadata.yaml').read_text())
        duration_s = metadata['rosbag2_bagfile_information']['duration']['nanoseconds']*1e-9
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError('bag 的录制时长无效')
        playback_timeout_s = max(180., duration_s/self.rate+30.)
        env = os.environ.copy()
        for key in ('RM_LIO_INIT_VOXEL', 'RM_LIO_PLANE_RANK_GUARD', 'RM_LIO_QOS_DEPTH'): env.pop(key, None)
        env['RM_LIO_PLANE_TRACE'] = str(self.output/'plane_diagnostics.csv')
        env['RM_LIO_INPUT_TRACE'] = str(self.output/'input_diagnostics.csv')
        if self.qos_depth is not None and not self.official_sensor_parameters:
            env['RM_LIO_QOS_DEPTH'] = str(self.qos_depth)
        if self.variant in ('init_voxel', 'both'): env['RM_LIO_INIT_VOXEL'] = '1'
        if self.variant in ('plane_guard', 'both'): env['RM_LIO_PLANE_RANK_GUARD'] = '1'
        lio = self.launch('point_lio', [str(self.executable), '--ros-args', '--params-file',
            str(self.output/'point_lio.yaml'), '-r', '__ns:='+NS+'/diagnosis'], env)
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if lio.poll() is not None: raise RuntimeError('Point-LIO 未能启动')
        playback_started = time.monotonic()
        player = self.launch('play', ['ros2', 'bag', 'play', str(self.source/'sensor_bag'),
            '--rate', str(self.rate), '--delay', '1', '--disable-keyboard-controls',
            '--topics', '/clock', NS+'/livox/lidar', NS+'/imu'])
        deadline = time.monotonic()+playback_timeout_s
        while player.poll() is None:
            rclpy.spin_once(self, timeout_sec=.02)
            if lio.poll() is not None: raise RuntimeError('Point-LIO 回放中退出')
            if time.monotonic() > deadline: raise TimeoutError('回放超时')
        if player.returncode != 0: raise RuntimeError('bag 回放失败')
        playback_wall_duration_s = time.monotonic()-playback_started
        gt = np.loadtxt(self.source/'gazebo_truth.csv', delimiter=',', ndmin=2)
        if (len(gt) < 2 or gt.shape[1] < 4 or not np.isfinite(gt).all()
                or np.any(np.diff(gt[:, 0]) <= 0)):
            raise ValueError('真值记录必须含至少两个有限且时间递增的位姿')
        deadline = time.monotonic()+60.
        # 大队列对照允许估计器排空已收到的记录，不能把落后的半段轨迹
        # 与整段原版直接比较。排空目标来自原始输入点，真值仅用于评分。
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            if self.rows and self.rows[-1][0] >= self.expected_last_stamp-DRAIN_TOLERANCE_S: break
            if lio.poll() is not None: raise RuntimeError('排空输入时Point-LIO退出')
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline: rclpy.spin_once(self, timeout_sec=.02)
        if len(self.rows) < 100: raise RuntimeError('回放没有足够的定位输出')
        rows = np.asarray(self.rows)
        rows = rows[(rows[:, 0] >= gt[0, 0]) & (rows[:, 0] <= gt[-1, 0])]
        if len(rows) < 100 or not np.isfinite(rows).all():
            raise RuntimeError('回放与真值没有足够的有限定位输出重叠')
        expected = np.column_stack([np.interp(rows[:, 0], gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
        expected -= np.array([0., 0., .7])
        errors = np.linalg.norm(rows[:, 1:4]-expected, axis=1)
        with (self.output/'trajectory.csv').open('w', newline='') as f:
            writer = csv.writer(f); writer.writerow(['t', 'x', 'y', 'z', 'yaw', 'truth_x', 'truth_y', 'truth_z', 'error_m'])
            for a, b, error in zip(rows, expected, errors): writer.writerow([*a, *b, error])
        return {'position_rmse_m': float(np.sqrt(np.mean(errors**2))), 'position_max_m': float(np.max(errors)),
                'input_drained_to_end': bool(self.rows[-1][0] >= self.expected_last_stamp-DRAIN_TOLERANCE_S),
                'expected_last_lidar_point_stamp': float(self.expected_last_stamp),
                'input_drain_tolerance_s': DRAIN_TOLERANCE_S,
                'drain_stamp_error_s': float(self.expected_last_stamp-self.rows[-1][0]),
                'recorded_duration_s': float(duration_s),
                'playback_timeout_s': float(playback_timeout_s),
                'playback_wall_duration_s': float(playback_wall_duration_s),
                'estimated_poses': len(rows), 'first_stamp': float(rows[0, 0]), 'last_stamp': float(rows[-1, 0]),
                'last_position_error_m': float(errors[-1])}

    def close(self):
        codes = {}
        for name, process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=8.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3.)
            codes[name] = process.returncode
        for handle in self.handles: handle.close()
        self.destroy_node()
        return codes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--executable', type=Path, default=Path('/opt/lio_sim/install/point_lio/lib/point_lio/pointlio_mapping'))
    parser.add_argument('--variant', choices=['installed', 'original', 'init_voxel', 'plane_guard', 'both'], default='installed')
    parser.add_argument('--rate', type=float, default=2.)
    parser.add_argument('--qos-depth', type=int, help='传感器队列深度；支持正式参数的镜像或自定义诊断二进制可用')
    parser.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('须设置 ROS_LOCALHOST_ONLY=1 和独立 ROS_DOMAIN_ID')
    if not .1 <= args.rate <= 4: parser.error('rate 须为0.1～4')
    if args.qos_depth is not None and not 1 <= args.qos_depth <= 8192: parser.error('qos-depth 须为1～8192')
    installed_executable = args.executable == parser.get_default('executable')
    if args.variant != 'installed' and installed_executable:
        parser.error('诊断变体必须指定已构建的诊断可执行文件')
    manifest = Path('/opt/lio_sim/plane_guard_manifest.json')
    image_build = (json.loads(manifest.read_text())
                   if installed_executable and manifest.exists() else {'variant': 'upstream'})
    if installed_executable:
        for filename, key in (('knn_manifest.json', 'nearest_neighbor_fix'),
                              ('measurement_noise_manifest.json', 'measurement_noise_fix')):
            extra = manifest.with_name(filename)
            if extra.exists(): image_build[key] = json.loads(extra.read_text())
    official_sensor_parameters = installed_executable and image_build.get('sensor_qos_depth_parameter') is True
    if args.qos_depth is not None and installed_executable and not official_sensor_parameters:
        parser.error('当前镜像安装版不支持 sensor_qos_depth；须更新镜像或指定自定义诊断二进制')
    source, output = args.source.resolve(), args.output_dir.resolve()
    if not (source/'sensor_bag/metadata.yaml').exists(): parser.error('源目录没有完整 sensor_bag')
    output.mkdir(parents=True, exist_ok=False)
    overrides = {key: yaml.safe_load(value) for key, value in (item.split('=', 1) for item in args.set)}
    report = {'source': str(source), 'variant': args.variant, 'overrides': overrides,
              'playback_rate': args.rate, 'ground_truth_used_as_estimator_input': False,
              'qos_depth_override': args.qos_depth,
              'sensor_configuration_mode': 'ros_parameters' if official_sensor_parameters else 'diagnostic_environment',
              'input_audit_path': str(output/'input_diagnostics.csv'),
              'executable': str(args.executable), 'executable_sha256': hashlib.sha256(args.executable.read_bytes()).hexdigest()}
    if installed_executable: report['point_lio_image_build'] = image_build
    rclpy.init(); app = Replay(source, output, args.executable, overrides, args.variant, args.rate,
                              args.qos_depth, official_sensor_parameters)
    try: report.update(app.run())
    except Exception as exc: report['error'] = repr(exc)
    finally:
        report['node_exit_codes'] = app.close()
        if rclpy.ok(): rclpy.shutdown()
    report['completed'] = 'error' not in report and all(code == 0 for code in report['node_exit_codes'].values())
    report['accuracy_passed'] = (report['completed'] and report['input_drained_to_end'] and report['position_rmse_m'] < .15
                                 and report['position_max_m'] < .35)
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
