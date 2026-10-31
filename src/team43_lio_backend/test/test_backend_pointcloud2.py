"""真实DDS验证固定PointCloud2接收、晚启动、remap与坏帧拒收。"""
import csv
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
from types import SimpleNamespace

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField


def cloud(index):
    msg = PointCloud2(height=1, width=1, point_step=26, row_step=26)
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = index * 1_000_000
    msg.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                  for name, offset, kind in (('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                                             ('tag', 16, 2), ('timestamp', 18, 8))]
    msg.data = struct.pack('<ffffBBd', 1., 0., 0., 100., 0x10, 0,
                           100_000_000_000. + index * 1_000_000.)
    return msg


@pytest.fixture
def trial(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    domain = 170 + os.getpid() % 40
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('pointcloud2_source', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    config = Path(get_package_share_directory('team43_lio_backend')) / 'config/small_point_lio_mid360.yaml'
    state = SimpleNamespace(node=node, process=None, topic='/test/pointcloud2/lidar',
                            imu_topic='/test/pointcloud2/imu', audit=tmp_path/'audit.csv',
                            log_path=tmp_path/'backend.log')

    def start(imu_reliable=False):
        # /clock未发布时也必须能连接晚启动的发布者，并正确解析相对话题remap。
        command = ['ros2', 'run', 'team43_lio_backend', 'small_point_lio_node', '--ros-args',
                   '--params-file', str(config), '-r', '__ns:=/test/pointcloud2',
                   '-r', '__node:=pointcloud2_backend', '-p', 'lidar_topic:=raw',
                   '-r', 'raw:='+state.topic, '-p', 'imu_topic:='+state.imu_topic,
                   '-p', 'use_sim_time:=true', '-p', 'input_audit_path:='+str(state.audit)]
        if imu_reliable: command += ['-p', 'imu_qos_reliable:=true']
        state.log = state.log_path.open('w')
        state.process = subprocess.Popen(command, stdout=state.log, stderr=subprocess.STDOUT,
                                         env=dict(os.environ, ROS_DOMAIN_ID=str(domain)),
                                         start_new_session=True)

    def wait(predicate):
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline:
            assert state.process.poll() is None, state.log_path.read_text()
            if predicate():
                return
            executor.spin_once(timeout_sec=.03)
        pytest.fail('等待PointCloud2接收超时：' + state.log_path.read_text())

    def close():
        if state.process is None:
            return
        if state.process.poll() is None:
            os.killpg(state.process.pid, signal.SIGINT)
        try:
            state.process.wait(timeout=10.)
        except subprocess.TimeoutExpired:
            os.killpg(state.process.pid, signal.SIGKILL)
            state.process.wait()
        state.log.close()
        assert state.process.returncode == 0, state.log_path.read_text()
        state.process = None

    state.start, state.wait, state.close = start, wait, close
    try:
        yield state
    finally:
        close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_fixed_pointcloud2_connects_late_publisher_without_clock_and_with_remap(trial):
    trial.start()
    trial.wait(lambda: '固定格式：PointCloud2' in trial.log_path.read_text())
    publisher = trial.node.create_publisher(PointCloud2, trial.topic, qos_profile_sensor_data)
    trial.wait(lambda: publisher.get_subscription_count() == 1)
    # DDS端点匹配可能早于ROS图事件送达，等待图更新而非立即读取一次。
    trial.wait(lambda: [e.topic_type for e in trial.node.get_subscriptions_info_by_topic(trial.topic)
                        if e.node_name == 'pointcloud2_backend'] == ['sensor_msgs/msg/PointCloud2'])
    for i in range(5):
        publisher.publish(cloud(i))
        time.sleep(.05)
    trial.close()
    with trial.audit.open() as stream:
        counts = next(csv.DictReader(stream))
    assert int(counts['lidar_received']) == 5


def test_invalid_cloud_does_not_enable_imu(trial):
    trial.start()
    publisher = trial.node.create_publisher(PointCloud2, trial.topic, qos_profile_sensor_data)
    imu = trial.node.create_publisher(Imu, trial.imu_topic, qos_profile_sensor_data)
    trial.wait(lambda: publisher.get_subscription_count() == imu.get_subscription_count() == 1)
    invalid = cloud(0)
    invalid.fields = [field for field in invalid.fields if field.name != 'timestamp']
    for _ in range(3):
        publisher.publish(invalid)
        imu.publish(Imu())
        time.sleep(.05)
    trial.wait(lambda: '拒收 Livox PointCloud2' in trial.log_path.read_text())
    trial.close()
    with trial.audit.open() as stream:
        counts = next(csv.DictReader(stream))
    assert int(counts['lidar_received']) == 0
    assert int(counts['imu_received']) == 0


def test_valid_cloud_after_bad_schema_is_received_in_same_process(trial):
    trial.start()
    publisher = trial.node.create_publisher(PointCloud2, trial.topic, qos_profile_sensor_data)
    trial.wait(lambda: publisher.get_subscription_count() == 1)
    invalid = cloud(0)
    invalid.fields = []
    def rejection_seen():
        # BEST_EFFORT的首帧不保证送达；等实际拒收证据，不把发现订阅等同于收包。
        publisher.publish(invalid)
        return '拒收 Livox PointCloud2' in trial.log_path.read_text()
    trial.wait(rejection_seen)
    for i in range(1, 4):
        publisher.publish(cloud(i))
        time.sleep(.05)
    trial.close()
    with trial.audit.open() as stream:
        counts = next(csv.DictReader(stream))
    assert int(counts['lidar_received']) == 3


def test_initial_imu_is_buffered_and_forwarded_after_first_valid_cloud(trial):
    trial.start(imu_reliable=True)
    publisher = trial.node.create_publisher(PointCloud2, trial.topic, qos_profile_sensor_data)
    imu = trial.node.create_publisher(Imu, trial.imu_topic, 10)
    trial.wait(lambda: publisher.get_subscription_count() == imu.get_subscription_count() == 1)
    # 先用坏帧确认点云通道实际送达，仍不能开启有效点云门；IMU用可靠传输，
    # 本用例验证后端缓存，不能把BEST_EFFORT首帧丢失误判成缓存丢包。
    invalid = cloud(0); invalid.fields = []
    def transport_ready():
        publisher.publish(invalid)
        return '拒收 Livox PointCloud2' in trial.log_path.read_text()
    trial.wait(transport_ready)
    for i in range(3):
        msg = Imu()
        msg.header.stamp.sec = 100
        msg.header.stamp.nanosec = i * 5_000_000
        msg.linear_acceleration.z = 1.
        imu.publish(msg)
        time.sleep(.05)
    from rclpy.duration import Duration
    assert imu.wait_for_all_acked(Duration(seconds=3.))
    publisher.publish(cloud(0))
    time.sleep(.15)
    trial.close()
    with trial.audit.open() as stream:
        counts = next(csv.DictReader(stream))
    assert int(counts['lidar_received']) == 1
    assert int(counts['imu_received']) == 3
    assert int(counts['imu_backward']) == 0
    assert float(counts['imu_first_stamp_s']) == 100.
    assert float(counts['imu_last_stamp_s']) == 100.01
