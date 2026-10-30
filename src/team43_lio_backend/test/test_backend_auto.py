"""真实 DDS 图发现：晚启动、类型冲突、恢复、remap 与选定后的格式改变。"""
import csv
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


@pytest.fixture
def trial(tmp_path):
    domain = 170 + os.getpid() % 40
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('auto_lidar_source', context=context)
    other_context = Context()
    rclpy.init(context=other_context, domain_id=domain)
    other_node = rclpy.create_node('auto_conflicting_source', context=other_context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    config = Path(get_package_share_directory('team43_lio_backend')) / 'config/small_point_lio_mid360.yaml'
    state = SimpleNamespace(node=node, other_node=other_node, topic='/test/auto/lidar', process=None,
                            output='', log_path=tmp_path/'backend.log', audit=tmp_path/'audit.csv')

    def start(relative_remap=False, sim_time=False):
        command = ['ros2', 'run', 'team43_lio_backend', 'small_point_lio_node', '--ros-args',
                   '--params-file', str(config), '-r', '__ns:=/test/auto', '-r', '__node:=auto_backend',
                   '-p', 'input_audit_path:='+str(state.audit),
                   '-p', 'lidar_topic:='+('raw' if relative_remap else state.topic)]
        # 不显式传 lidar_format，验证配置默认 auto。
        if relative_remap:
            command += ['-r', 'raw:='+state.topic]
        if sim_time:
            command += ['-p', 'use_sim_time:=true']
        state.log = state.log_path.open('w')
        state.process = subprocess.Popen(command, stdout=state.log, stderr=subprocess.STDOUT,
                                         env=dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1'),
                                         start_new_session=True)

    def wait(predicate, timeout=10.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if state.process:
                assert state.process.poll() is None, state.log_path.read_text()
            if predicate():
                return
            executor.spin_once(timeout_sec=.03)
        pytest.fail('等待自动识别超时：'+state.log_path.read_text())

    def readers():
        return [e.topic_type for e in node.get_subscriptions_info_by_topic(state.topic)
                if e.node_name == 'auto_backend']

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
        state.output = state.log_path.read_text()
        assert state.process.returncode == 0, state.output
        state.process = None

    state.start, state.wait, state.readers, state.close = start, wait, readers, close
    try:
        yield state
    finally:
        close()
        executor.shutdown()
        node.destroy_node()
        other_node.destroy_node()
        context.shutdown()
        other_context.shutdown()


@pytest.mark.parametrize('message_type,expected', [
    (PointCloud2, 'sensor_msgs/msg/PointCloud2'), (CustomMsg, 'livox_ros_driver2/msg/CustomMsg')])
def test_auto_waits_for_late_publisher_and_resolves_relative_remap(trial, message_type, expected):
    # /clock 尚未发布也必须继续自动发现，不依赖仿真时间推进。
    trial.start(relative_remap=True, sim_time=True)
    trial.wait(lambda: '等待点云发布者' in trial.log_path.read_text())
    assert not trial.readers()
    publisher = trial.node.create_publisher(message_type, trial.topic, qos_profile_sensor_data)
    trial.wait(lambda: trial.readers() == [expected])
    trial.wait(lambda: publisher.get_subscription_count() == 1)
    assert '自动识别并锁定' in trial.log_path.read_text()


@pytest.mark.parametrize('conflict', [False, True])
def test_auto_rejects_unsupported_or_conflicting_types_then_recovers(trial, conflict):
    invalid = trial.node.create_publisher(Imu, trial.topic, qos_profile_sensor_data)
    valid = None
    if conflict:
        valid = trial.other_node.create_publisher(CustomMsg, trial.topic, qos_profile_sensor_data)
    trial.start(sim_time=True)
    trial.wait(lambda: '类型不支持或存在冲突' in trial.log_path.read_text())
    assert not trial.readers()
    trial.node.destroy_publisher(invalid)
    if valid is None:
        valid = trial.node.create_publisher(CustomMsg, trial.topic, qos_profile_sensor_data)
    trial.wait(lambda: trial.readers() == ['livox_ros_driver2/msg/CustomMsg'])


def test_auto_stops_accepting_after_selected_type_conflict_even_if_it_clears(trial):
    publisher = trial.node.create_publisher(CustomMsg, trial.topic, qos_profile_sensor_data)
    trial.start()
    trial.wait(lambda: publisher.get_subscription_count() == 1)

    def send(index):
        msg = CustomMsg()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = index*1000000
        msg.timebase = 1000000000+index*1000000
        msg.points = [CustomPoint(x=1., y=0., z=0., tag=0x10)]
        msg.point_num = 1
        publisher.publish(msg)
        time.sleep(.05)

    for i in range(5):
        send(i)
    conflict = trial.other_node.create_publisher(PointCloud2, trial.topic, qos_profile_sensor_data)
    trial.wait(lambda: '点云类型冲突或改变，停止接收' in trial.log_path.read_text())
    for i in range(5, 10):
        send(i)
    trial.other_node.destroy_publisher(conflict)
    for i in range(10, 15):
        send(i)
    trial.close()
    with trial.audit.open() as stream:
        counts = next(csv.DictReader(stream))
    assert 1 <= int(counts['lidar_received']) <= 5, counts
    assert '需重启定位' in trial.output
