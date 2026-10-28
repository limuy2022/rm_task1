"""`team43_dataio/trajectory_annotator_node.py` 的功能性回归（真 ROS2 Humble）。

手法：**真 rclpy 节点** + `monkeypatch.chdir(tmp_path)` 重定向输出 +
真 `TargetState` / `Odometry` / `OccupancyGrid` 消息直调回调、
`shutdown()` 触发 `team43_grid_mapper.png_draw.draw_tracked` 真实画图落盘。

⚠ 这里验证的是**节点逻辑**（轨迹缓存、抽稀、grid 解析、PNG 产物）；
DDS 传输与真实进程退出钩子不在本文件范围。
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip('rclpy')

if rclpy is not None:
    from nav_msgs.msg import Odometry, OccupancyGrid
    from team43_interfaces.msg import TargetState

    from team43_dataio.trajectory_annotator_node import TrajectoryAnnotatorNode


@pytest.fixture()
def ros():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _target(cur=(2.0, 1.0), pred=(2.4, 1.2), valid=True):
    msg = TargetState()
    msg.valid = bool(valid)
    msg.cur_x, msg.cur_y = float(cur[0]), float(cur[1])
    msg.pred_x, msg.pred_y = float(pred[0]), float(pred[1])
    return msg


def test_state_subscriptions_accept_best_effort_publishers(ros):
    """检查真实节点创建的订阅配置；不把直调回调当成 DDS 通信验证。"""
    from rclpy.qos import QoSCompatibility, qos_check_compatible
    from team43_common.qos_utils import sensor_qos
    node = TrajectoryAnnotatorNode()
    try:
        subscriptions = {sub.topic_name: sub.qos_profile for sub in node.subscriptions}
        for topic in ('/estimation/target_state', '/gimbal/base_pose'):
            # liveliness 两侧均为 SYSTEM_DEFAULT 时 rclpy 必然给 WARNING（取决于
            # RMW 默认）；要拦的是 RELIABLE 订阅拒配 BEST_EFFORT 发布的 ERROR。
            compatibility, reason = qos_check_compatible(sensor_qos(), subscriptions[topic])
            assert compatibility != QoSCompatibility.ERROR, reason
    finally:
        node.destroy_node()


def _base(x=0.5, y=-0.5):
    msg = Odometry()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    return msg


def _grid(width=8, height=6, resolution=0.5, origin=(-2.0, -1.5)):
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = float(resolution)
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    # 三态：-1 未知 / 0 自由 / 100 占据（中间一行占据，其余自由）
    msg.data = [0] * (width * height)
    for col in range(width):
        msg.data[2 * width + col] = 100
    return msg


def test_no_grid_skips_png(tmp_path, monkeypatch, ros):
    """未收到 /map/grid 时 shutdown 跳过 PNG，不得抛错。"""
    monkeypatch.chdir(tmp_path)
    node = TrajectoryAnnotatorNode()
    node._on_target(_target())
    node._on_base_pose(_base())
    node.shutdown()
    node.destroy_node()
    assert not (tmp_path / 'outputs' / 'latest' / 'map_tracked.png').exists()


def test_grid_and_tracks_write_png(tmp_path, monkeypatch, ros):
    monkeypatch.chdir(tmp_path)
    node = TrajectoryAnnotatorNode()
    try:
        node._on_grid(_grid())
        assert node._grid is not None, 'OccupancyGrid 解析失败'
        for k in range(5):
            node._on_target(_target(cur=(2.0 + 0.1 * k, 1.0)))
            node._on_base_pose(_base(0.5 + 0.05 * k, -0.5))
        assert len(node._target_traj) == 5
        assert len(node._base_traj) == 5
        assert node._target_pred == (2.4, 1.2)
    finally:
        node.shutdown()
        node.destroy_node()
    png = tmp_path / 'outputs' / 'latest' / 'map_tracked.png'
    assert png.is_file() and png.stat().st_size > 0


def test_invalid_target_is_ignored(tmp_path, monkeypatch, ros):
    monkeypatch.chdir(tmp_path)
    node = TrajectoryAnnotatorNode()
    try:
        node._on_target(_target(valid=False))
        node._on_target(_target(cur=(float('nan'), 1.0)))
        assert node._target_traj == []
        assert node._target_cur is None
        node._on_base_pose(_base(x=float('inf')))
        assert node._base_traj == []
    finally:
        node.shutdown()
        node.destroy_node()


def test_max_points_thins_trajectory(tmp_path, monkeypatch, ros):
    """轨迹缓存超过上限时抽稀一半。"""
    monkeypatch.chdir(tmp_path)
    node = TrajectoryAnnotatorNode()
    try:
        node._max_points = 4
        for k in range(10):
            node._on_target(_target(cur=(float(k), 0.0)))
        assert len(node._target_traj) <= 4, '超限后必须抽稀'
        assert len(node._target_traj) >= 2, '抽稀不得清空轨迹'
        xs = [p[0] for p in node._target_traj]
        assert xs == sorted(xs), '抽稀保持时间顺序'
    finally:
        node.shutdown()
        node.destroy_node()
