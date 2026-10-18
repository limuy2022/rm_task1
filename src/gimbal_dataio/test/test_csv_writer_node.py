"""`gimbal_dataio/csv_writer_node.py` 的功能性回归（真 ROS2 Humble）。

手法：**真 rclpy 节点**（`CsvWriterNode` 直接构造，`rclpy.init()` 一次）+
`monkeypatch.chdir(tmp_path)` 把 `outputs/<run_id>` 重定向到临时目录 +
真 `TargetState` / `Odometry` / `ChassisAdvice` 消息直调回调、手动触发
`_on_timer`（不依赖 spin）。落盘格式与收尾产物（result.csv /
result_with_header.csv / run_meta.json）用文件内容断言。

列序（COLS8）：pred_x, pred_y, cur_x, cur_y, base_x, base_y, cmd_vx, cmd_vy；
timestamp / track_status 只在 extra_cols 时追加。

⚠ 这里验证的是**节点逻辑**：CSV 写盘器模块本身的边界（NaN 守卫、列宽、
重启去重）在 test_csv_writer.py；DDS 传输与真实 20Hz 定时器不在本文件范围。
"""

from __future__ import annotations

import csv
import json

import pytest

rclpy = pytest.importorskip('rclpy')

if rclpy is not None:
    from nav_msgs.msg import Odometry
    from gimbal_interfaces.msg import ChassisAdvice, TargetState

    from gimbal_dataio.csv_writer_node import CsvWriterNode


@pytest.fixture()
def ros():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _target(valid=True, pred=(1.5, -0.5), cur=(1.0, 0.2), track_status=2):
    msg = TargetState()
    msg.valid = bool(valid)
    msg.pred_x, msg.pred_y = float(pred[0]), float(pred[1])
    msg.cur_x, msg.cur_y = float(cur[0]), float(cur[1])
    msg.track_status = int(track_status)
    return msg


def _base(x=3.0, y=-1.0):
    msg = Odometry()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    return msg


def _advice(vx=0.4, vy=-0.1):
    msg = ChassisAdvice()
    msg.cmd_vx, msg.cmd_vy = float(vx), float(vy)
    return msg


# ---------------------------------------------------------------------------
# 1. 写盘门控：首个有效估计之前不写行
# ---------------------------------------------------------------------------
def test_no_rows_before_first_valid_estimate(tmp_path, monkeypatch, ros):
    monkeypatch.chdir(tmp_path)
    node = CsvWriterNode()
    try:
        for _ in range(5):
            node._on_timer()
        assert node._writer.row_count == 0, 'write_after_first_estimate=True 时不得提前写盘'

        # 无效估计不算数
        node._on_target(_target(valid=False))
        node._on_timer()
        assert node._writer.row_count == 0

        node._on_target(_target())
        node._on_timer()
        assert node._writer.row_count == 1
        assert node._t0 is not None
    finally:
        node.shutdown()
        node.destroy_node()


# ---------------------------------------------------------------------------
# 2. 行内容：8 列、数值按回调缓存写入
# ---------------------------------------------------------------------------
def test_rows_carry_subscribed_values(tmp_path, monkeypatch, ros):
    monkeypatch.chdir(tmp_path)
    node = CsvWriterNode()
    try:
        node._on_target(_target(pred=(1.5, -0.5), cur=(1.0, 0.2)))
        node._on_base_pose(_base(3.0, -1.0))
        node._on_advice(_advice(0.4, -0.1))
        for _ in range(3):
            node._on_timer()
        node._writer.flush()          # flush_period_s=1s，手动刷盘后再读文件

        path = tmp_path / 'outputs' / 'latest' / 'result.csv'
        assert path.is_file()
        rows = list(csv.reader(path.read_text(encoding='utf-8').splitlines()))
        assert len(rows) == 3
        for row in rows:
            assert len(row) == 8, '默认严格 8 列'
            assert float(row[0]) == pytest.approx(1.5)   # pred_x
            assert float(row[1]) == pytest.approx(-0.5)  # pred_y
            assert float(row[2]) == pytest.approx(1.0)   # cur_x
            assert float(row[3]) == pytest.approx(0.2)   # cur_y
            assert float(row[4]) == pytest.approx(3.0)   # base_x
            assert float(row[5]) == pytest.approx(-1.0)  # base_y
            assert float(row[6]) == pytest.approx(0.4)   # cmd_vx
            assert float(row[7]) == pytest.approx(-0.1)  # cmd_vy
    finally:
        node.shutdown()
        node.destroy_node()


# ---------------------------------------------------------------------------
# 3. 数据未齐仍写盘（当前值兜底）
# ---------------------------------------------------------------------------
def test_partial_data_still_writes(tmp_path, monkeypatch, ros):
    monkeypatch.chdir(tmp_path)
    node = CsvWriterNode()
    try:
        node._on_target(_target())          # 只有目标，没有底盘/建议
        node._on_timer()
        assert node._writer.row_count == 1
        assert node._has_base is False and node._has_cmd is False
    finally:
        node.shutdown()
        node.destroy_node()


# ---------------------------------------------------------------------------
# 4. shutdown 产物
# ---------------------------------------------------------------------------
def test_shutdown_writes_artifacts(tmp_path, monkeypatch, ros):
    monkeypatch.chdir(tmp_path)
    node = CsvWriterNode()
    node._on_target(_target())
    node._on_base_pose(_base())
    node._on_advice(_advice())
    for _ in range(4):
        node._on_timer()
    node.shutdown()
    node.destroy_node()

    run_dir = tmp_path / 'outputs' / 'latest'
    result = run_dir / 'result.csv'
    with_header = run_dir / 'result_with_header.csv'
    meta = run_dir / 'run_meta.json'
    assert result.is_file() and with_header.is_file() and meta.is_file()

    rows = list(csv.reader(result.read_text(encoding='utf-8').splitlines()))
    assert len(rows) == 4 and all(len(r) == 8 for r in rows)

    header_rows = list(csv.reader(with_header.read_text(encoding='utf-8').splitlines()))
    assert len(header_rows) == len(rows) + 1, '带表头副本应多一行'

    data = json.loads(meta.read_text(encoding='utf-8'))
    assert data['params']['csv.rate_hz'] == pytest.approx(20.0)
    assert any(a['path'].endswith('result.csv') for a in data['artifacts'])
