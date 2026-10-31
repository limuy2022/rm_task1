"""通过真实后端的 DDS 端点确认默认 QoS 与仿真可靠 IMU 选项。"""
import os
import signal
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy


@pytest.mark.parametrize('reliable', [False, True])
def test_backend_imu_reliability_is_explicit_opt_in(reliable, monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    domain = 170 + os.getpid() % 40
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('backend_qos_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    config = Path(get_package_share_directory('team43_lio_backend')) / 'config/small_point_lio_mid360.yaml'
    command = ['ros2', 'run', 'team43_lio_backend', 'small_point_lio_node',
               '--ros-args', '--params-file', str(config),
               '-r', '__node:=qos_backend', '-r', '__ns:=/test/lio_qos',
               '-p', 'imu_topic:=/test/lio_qos/imu',
               '-p', 'lidar_topic:=/test/lio_qos/lidar']
    if reliable:
        command += ['-p', 'imu_qos_reliable:=true']
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               env=dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1'),
                               start_new_session=True, text=True)
    try:
        deadline = time.monotonic() + 15.
        endpoints = {}
        while time.monotonic() < deadline:
            assert process.poll() is None, '后端在发现 QoS 前退出'
            endpoints = {topic: [e for e in node.get_subscriptions_info_by_topic('/test/lio_qos/'+topic)
                                 if e.node_name == 'qos_backend'] for topic in ('imu', 'lidar')}
            if all(endpoints.values()):
                break
            executor.spin_once(timeout_sec=.05)
        assert all(endpoints.values()), endpoints
        expected = ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
        assert endpoints['imu'][0].qos_profile.reliability == expected
        assert endpoints['lidar'][0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        expected_type = 'sensor_msgs/msg/PointCloud2'
        assert endpoints['lidar'][0].topic_type == expected_type
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=10.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    assert process.returncode == 0, output


def test_backend_produces_static_localization_with_pointcloud2(tmp_path):
    """合成点云经真实 DDS/外参/算法核心输出位姿和地图，防止只接通端点。"""
    script = Path(__file__).resolve().parents[3] / 'tools/verify_lio_install.py'
    result = subprocess.run(['python3', '-B', str(script), '--output-dir', str(tmp_path / 'pointcloud2')],
                            env=dict(os.environ, ROS_DOMAIN_ID=str(170 + os.getpid() % 40),
                                     ROS_LOCALHOST_ONLY='1'),
                            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=45.)
    assert result.returncode == 0, result.stdout
