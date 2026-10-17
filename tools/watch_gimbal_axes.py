"""只订阅现有云台IMU，以4Hz显示三轴与姿态；不打开串口、不发布控制或TF。"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time


def orientation_values(q_xyzw, available=True):
    """返回ZYX欧拉角和静止比力预测；q表示G到world的旋转。"""
    values = tuple(float(v) for v in q_xyzw)
    if not available or not all(math.isfinite(v) for v in values):
        return None
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm < .5:
        return None
    x, y, z, w = (v/norm for v in values)
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    gravity = 9.80665
    return {
        'q_norm': norm,
        'rpy_deg': [math.degrees(v) for v in (roll, pitch, yaw)],
        'rest_specific_force_from_q': [
            gravity*2*(x*z-w*y),
            gravity*2*(y*z+w*x),
            gravity*(1-2*(x*x+y*y)),
        ],
    }


def format_sample(sample):
    def vec(values):
        return '('+', '.join(f'{v:+.3f}' for v in values)+')'
    line = (f"a={vec(sample['accel_xyz'])} m/s2  "
            f"w={vec(sample['gyro_xyz'])} rad/s")
    orientation = sample['orientation']
    if orientation is None:
        return line+'  姿态未提供/无效'
    roll, pitch, yaw = orientation['rpy_deg']
    return line+f'  roll={roll:+.1f} pitch={pitch:+.1f} yaw={yaw:+.1f} deg'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--label', default='g_axes')
    parser.add_argument('--seconds', type=float, default=30.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 600:
        parser.error('--seconds必须在1到600秒之间')
    if not args.label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.label):
        parser.error('--label仅使用英文字母、数字、下划线或短横线')
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
    except ImportError as exc:
        print(f'ROS环境没有加载：{exc}。先source Humble；不安装依赖。')
        return 2
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir/(args.label+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')
    rclpy.init()
    node = Node('gimbal_axes_view', namespace='team47_check')
    latest = [None]
    count = [0]

    def receive(msg):
        count[0] += 1
        accel = [float(getattr(msg.linear_acceleration, a)) for a in 'xyz']
        gyro = [float(getattr(msg.angular_velocity, a)) for a in 'xyz']
        quat = [float(getattr(msg.orientation, a)) for a in ('x','y','z','w')]
        if not all(math.isfinite(v) for v in accel+gyro):
            return
        latest[0] = {
            'received_monotonic': time.monotonic(),
            'stamp_sec': int(msg.header.stamp.sec),
            'stamp_nanosec': int(msg.header.stamp.nanosec),
            'frame_id': str(msg.header.frame_id),
            'accel_xyz': accel, 'gyro_xyz': gyro,
            'q_xyzw': quat if all(math.isfinite(v) for v in quat) else None,
            'orientation': orientation_values(quat, msg.orientation_covariance[0] != -1),
        }

    node.create_subscription(Imu, args.topic, receive, qos_profile_sensor_data)
    print(f'只读订阅：{args.topic}；每行xyz顺序固定。记录：{output}', flush=True)
    print('先静止；出现数据后再按教程做动作。a为比力，w为三轴角速度；姿态角不是机械关节角。', flush=True)
    started = time.monotonic()
    next_print = started
    last_stamp = None
    last_warning = started
    written = 0
    try:
        with output.open('x', encoding='utf-8') as stream:
            while rclpy.ok() and time.monotonic()-started < args.seconds:
                rclpy.spin_once(node, timeout_sec=.02)
                now = time.monotonic()
                sample = latest[0]
                if sample is None or now-sample['received_monotonic'] > 1.:
                    if now-last_warning >= 2.:
                        print('尚未收到新IMU数据；先查发布者/链路，不据此判轴。', flush=True)
                        last_warning = now
                    continue
                stamp = sample['stamp_sec'], sample['stamp_nanosec']
                if now < next_print or stamp == last_stamp:
                    continue
                if sample['frame_id'] != 'gimbal_link' or stamp == (0,0):
                    print(f"数据不符合本次预期：frame={sample['frame_id']!r}, stamp={stamp}；保存记录，先不要判轴。", flush=True)
                print(format_sample(sample), flush=True)
                stream.write(json.dumps(sample, ensure_ascii=False, allow_nan=False)+'\n')
                stream.flush()
                written += 1
                last_stamp = stamp
                next_print = now+.25
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'结束：收到回调{count[0]}次，保存不同stamp的显示样本{written}条；文件={output}', flush=True)
    return 0 if written >= 2 else 1


if __name__ == '__main__':
    raise SystemExit(main())
