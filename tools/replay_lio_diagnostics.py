#!/usr/bin/env python3
"""同一仿真 bag 的 Point-LIO 对照回放，保留测量时间并独立对照真值。

只回放 clock、Livox 和 IMU，不回放旧定位、TF、图像或目标结果。
预编译的诊断二进制通过 --executable 指定；默认使用镜像原版。
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
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
import yaml


NS = '/sim/gazebo'


class Replay(Node):
    def __init__(self, source, output, executable, overrides, variant, rate, qos_depth):
        super().__init__('lio_diagnostic_replay', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.source, self.output, self.executable = source, output, executable
        self.rows, self.handles, self.processes = [], [], []
        self.rate, self.variant = rate, variant
        self.qos_depth = qos_depth
        self.create_subscription(Odometry, NS+'/diagnosis/aft_mapped_to_init', self.observe, qos_profile_sensor_data)
        config = yaml.safe_load((source/'point_lio.yaml').read_text())
        params = config['/**']['ros__parameters']
        for key, value in overrides.items():
            obj = params
            parts = key.split('.')
            for part in parts[:-1]: obj = obj[part]
            if parts[-1] not in obj: raise ValueError('未知参数 '+key)
            obj[parts[-1]] = value
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
        env = os.environ.copy()
        for key in ('RM_LIO_INIT_VOXEL', 'RM_LIO_PLANE_RANK_GUARD', 'RM_LIO_QOS_DEPTH'): env.pop(key, None)
        env['RM_LIO_PLANE_TRACE'] = str(self.output/'plane_diagnostics.csv')
        env['RM_LIO_INPUT_TRACE'] = str(self.output/'input_diagnostics.csv')
        if self.qos_depth is not None: env['RM_LIO_QOS_DEPTH'] = str(self.qos_depth)
        if self.variant in ('init_voxel', 'both'): env['RM_LIO_INIT_VOXEL'] = '1'
        if self.variant in ('plane_guard', 'both'): env['RM_LIO_PLANE_RANK_GUARD'] = '1'
        lio = self.launch('point_lio', [str(self.executable), '--ros-args', '--params-file',
            str(self.output/'point_lio.yaml'), '-r', '__ns:='+NS+'/diagnosis'], env)
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if lio.poll() is not None: raise RuntimeError('Point-LIO 未能启动')
        player = self.launch('play', ['ros2', 'bag', 'play', str(self.source/'sensor_bag'),
            '--rate', str(self.rate), '--delay', '1', '--disable-keyboard-controls',
            '--topics', '/clock', NS+'/livox/lidar', NS+'/imu'])
        deadline = time.monotonic()+180.
        while player.poll() is None:
            rclpy.spin_once(self, timeout_sec=.02)
            if lio.poll() is not None: raise RuntimeError('Point-LIO 回放中退出')
            if time.monotonic() > deadline: raise TimeoutError('回放超时')
        if player.returncode != 0: raise RuntimeError('bag 回放失败')
        gt = np.loadtxt(self.source/'gazebo_truth.csv', delimiter=',')
        deadline = time.monotonic()+60.
        # 大队列对照允许估计器排空已收到的记录，不能把落后的半段轨迹
        # 与整段原版直接比较。真值只用于确定记录的结束窗口，不输入估计。
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            if self.rows and self.rows[-1][0] >= gt[-1, 0]-.05: break
            if lio.poll() is not None: raise RuntimeError('排空输入时Point-LIO退出')
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline: rclpy.spin_once(self, timeout_sec=.02)
        if len(self.rows) < 100: raise RuntimeError('回放没有足够的定位输出')
        rows = np.asarray(self.rows)
        rows = rows[(rows[:, 0] >= gt[0, 0]) & (rows[:, 0] <= gt[-1, 0])]
        expected = np.column_stack([np.interp(rows[:, 0], gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
        expected -= np.array([0., 0., .7])
        errors = np.linalg.norm(rows[:, 1:4]-expected, axis=1)
        with (self.output/'trajectory.csv').open('w', newline='') as f:
            writer = csv.writer(f); writer.writerow(['t', 'x', 'y', 'z', 'yaw', 'truth_x', 'truth_y', 'truth_z', 'error_m'])
            for a, b, error in zip(rows, expected, errors): writer.writerow([*a, *b, error])
        return {'position_rmse_m': float(np.sqrt(np.mean(errors**2))), 'position_max_m': float(np.max(errors)),
                'input_drained_to_end': bool(rows[-1, 0] >= gt[-1, 0]-.05),
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
    parser.add_argument('--variant', choices=['original', 'init_voxel', 'plane_guard', 'both'], default='original')
    parser.add_argument('--rate', type=float, default=2.)
    parser.add_argument('--qos-depth', type=int, help='诊断二进制的传感器队列深度；原版默认为5')
    parser.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('须设置 ROS_LOCALHOST_ONLY=1 和独立 ROS_DOMAIN_ID')
    if not .1 <= args.rate <= 4: parser.error('rate 须为0.1～4')
    if args.qos_depth is not None and not 1 <= args.qos_depth <= 8192: parser.error('qos-depth 须为1～8192')
    if (args.variant != 'original' or args.qos_depth is not None) and args.executable == parser.get_default('executable'):
        parser.error('诊断变体必须指定已构建的诊断可执行文件')
    source, output = args.source.resolve(), args.output_dir.resolve()
    if not (source/'sensor_bag/metadata.yaml').exists(): parser.error('源目录没有完整 sensor_bag')
    output.mkdir(parents=True, exist_ok=False)
    overrides = {key: yaml.safe_load(value) for key, value in (item.split('=', 1) for item in args.set)}
    report = {'source': str(source), 'variant': args.variant, 'overrides': overrides,
              'playback_rate': args.rate, 'ground_truth_used_as_estimator_input': False,
              'qos_depth_override': args.qos_depth,
              'executable': str(args.executable), 'executable_sha256': hashlib.sha256(args.executable.read_bytes()).hexdigest()}
    rclpy.init(); app = Replay(source, output, args.executable, overrides, args.variant, args.rate, args.qos_depth)
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
