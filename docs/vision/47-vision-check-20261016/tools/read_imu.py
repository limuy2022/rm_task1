"""只订阅一个sensor_msgs/Imu，最多8秒；不打开串口，不发送控制。"""
import argparse
import math
import statistics
import time


def norm(values):
    return math.sqrt(sum(x * x for x in values))


def summarize(samples):
    accel = [norm(s[1]) for s in samples]
    gyro = [norm(s[2]) for s in samples]
    return statistics.median(accel), statistics.median(gyro)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('topic')
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    rclpy.init()
    node = Node('imu_reader', namespace='team47_check')
    samples, seen = [], set()

    def receive(msg):
        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if stamp == (0, 0) or stamp in seen:
            return
        a = tuple(float(getattr(msg.linear_acceleration, k)) for k in ('x', 'y', 'z'))
        g = tuple(float(getattr(msg.angular_velocity, k)) for k in ('x', 'y', 'z'))
        q = tuple(float(getattr(msg.orientation, k)) for k in ('x', 'y', 'z', 'w'))
        if not all(math.isfinite(x) for x in a + g):
            return
        seen.add(stamp)
        samples.append((stamp, a, g, q, str(msg.header.frame_id), msg.orientation_covariance[0]))

    node.create_subscription(Imu, args.topic, receive, qos_profile_sensor_data)
    print(f'正在读取 {args.topic}，请保持设备静止，最多等待8秒……', flush=True)
    deadline = time.monotonic() + 8
    try:
        while len(samples) < 30 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(samples) < 2:
            print('未得到连续、非零且不同时间戳的有效IMU消息。保存topic info结果；不要重启驱动。')
            return 1
        a, g = summarize(samples)
        stamp, xyz, _, q, frame, unavailable = samples[-1]
        print(f'已收到 {len(samples)} 条不同时间戳消息；frame_id={frame}')
        print(f'最后时间戳：{stamp[0]}.{stamp[1]:09d}')
        print('最后加速度xyz：' + ', '.join(f'{v:.5f}' for v in xyz))
        print(f'加速度模长中位数：{a:.5f}；角速度模长中位数：{g:.5f}')
        if 0.7 < a < 1.3:
            print('加速度数值约1：与g单位相符，仍须核对驱动定义。')
        elif 7 < a < 13:
            print('加速度数值约9.8：与m/s²单位相符，仍须核对驱动定义。')
        else:
            print('静止模长不在约1或约9.8附近：先保存原始消息，不据此配置LIO。')
        if unavailable == -1 or not all(math.isfinite(v) for v in q) or norm(q) < 0.5:
            print('姿态：未提供有效四元数；雷达IMU可能正常只提供加速度/角速度。')
        else:
            print(f'四元数模长：{norm(q):.5f}，应接近1；这不是机械零位证明。')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
