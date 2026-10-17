"""`gimbal_debug/gt_error_monitor_node.py` 的功能性回归（**无需 ROS2**）。

手法：把 `rclpy` 与消息类型换成桩（`conftest.StubRos`），用**假节点 + 假单调时钟**
真跑 `GtErrorMonitorNode`：喂真值（world 系）与 `/gimbal/state`（quat = world→gimbal_link），
推进虚拟时间，检查 `e_ang` 数值、发布序列，以及退出时的 `eval_report.md` /
`eval_error_curve.csv` / `eval_metrics.json` 内容。

验证依据：PRD §6.1（`e_ang = angle(p_target_true − p_gimbal, 相机光轴方向)`，world 系）、
DESIGN §3.12、hardware_spec §5（RMS ≤ 2.0° / max ≤ 5.0° / 收敛 ≤ 2.0s / ≥ 50 样本）。

⚠ 这里验证的是**节点逻辑**。真实 `rclpy` 的 QoS、`header.stamp` 时钟域、
`colcon build` 与实跑 30s 验收仍需在 Ubuntu 24.04 + ROS 2 Jazzy 上做。
"""

from __future__ import annotations

import json
import math
import os
from typing import Any
from typing import Dict
from typing import List

import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.abspath(os.path.join(TEST_DIR, '..'))
SRC_DIR = os.path.abspath(os.path.join(PKG_DIR, '..'))
CONFIG_DIR = os.path.join(SRC_DIR, 'gimbal_tracker_bringup', 'config')
VIRTUAL_YAML = os.path.join(CONFIG_DIR, 'virtual.yaml')

GT_TOPIC_PARAM = 'debug.gt_topic'


# ---------------------------------------------------------------------------
# 假节点 / 假时钟
# ---------------------------------------------------------------------------
class _Clock(object):
    """可控单调时钟（节点内所有时间都走 `time_utils.now()`）。"""

    def __init__(self, t0: float = 1000.0) -> None:
        self.t = float(t0)

    def now(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


class _Param(object):
    def __init__(self, value: Any) -> None:
        self.value = value


class _Publisher(object):
    def __init__(self, sink: List[Any], topic: str) -> None:
        self._sink = sink
        self._topic = topic

    def publish(self, msg: Any) -> None:
        self._sink.append((self._topic, msg))


class _ClockHandle(object):
    def now(self) -> Any:
        return self

    def to_msg(self) -> str:
        return 'stub-stamp'


class _FakeNode(object):
    """实现节点用到的那一小部分 rclpy API。"""

    def __init__(self, overrides: Dict[str, Any]) -> None:
        self._params = dict(overrides)
        self.subs: Dict[str, Any] = {}
        self.pubs: Dict[str, Any] = {}
        self.timers: List[Any] = []
        self.published: List[Any] = []

    # -- 参数
    def declare_parameter(self, name: str, default: Any = None) -> Any:
        self._params.setdefault(name, default)
        return self._params[name]

    def has_parameter(self, name: str) -> bool:
        return name in self._params

    def get_parameter(self, name: str) -> _Param:
        return _Param(self._params.get(name))

    # -- 通信
    def create_subscription(self, msg_type: Any, topic: str, cb: Any, qos: Any) -> Any:
        self.subs[topic] = cb
        return object()

    def create_publisher(self, msg_type: Any, topic: str, qos: Any) -> Any:
        pub = _Publisher(self.published, topic)
        self.pubs[topic] = pub
        return pub

    def create_timer(self, period: float, cb: Any) -> Any:
        self.timers.append((float(period), cb))
        return object()

    # -- 杂项
    def get_clock(self) -> Any:
        return _ClockHandle()

    def get_name(self) -> str:
        return 'gt_error_monitor_node'

    def get_logger(self) -> Any:
        import logging

        class _Proxy(object):
            """让 `warn()` 走 `warning()`（对齐 rclpy 语义，消除 DeprecationWarning）。"""

            def __init__(self, inner: Any) -> None:
                self._inner = inner

            def warn(self, *a: Any, **kw: Any) -> Any:
                return self._inner.warning(*a, **kw)

            def __getattr__(self, item: str) -> Any:
                return getattr(self._inner, item)

        return _Proxy(logging.getLogger('gt_error_monitor_node'))

    # -- 便捷
    @property
    def gt_topic(self) -> str:
        return str(self._params[GT_TOPIC_PARAM])

    def errors(self, topic: str = '/debug/angle_error_deg') -> List[float]:
        return [m.data for tp, m in self.published if tp == topic]

    def paths(self, topic: str = '/debug/target_path') -> List[Any]:
        return [m for tp, m in self.published if tp == topic]


def _boot(stub_ros, tmp_path, **param_over: Any):
    """导入节点模块（桩生效）、替换时间源、构造节点。

    Returns:
        tuple: `(node_module, app, fake_node, clock)`
    """
    import gimbal_debug.gt_error_monitor_node as NODE

    clock = _Clock()
    NODE.now = clock.now                     # monkeypatch 模块内唯一时间源

    overrides = dict(NODE.PARAM_DEFAULTS)
    overrides.update({
        'config.virtual_yaml': VIRTUAL_YAML,
        'csv.out_dir': str(tmp_path),
        'csv.run_id': 'latest',
    })
    overrides.update(param_over)

    fake = _FakeNode(overrides)
    app = NODE.GtErrorMonitorNode(fake)
    return NODE, app, fake, clock


def _tick(fake: _FakeNode, clock: _Clock, n: int = 1, dt: float = 0.01) -> None:
    """推进时间并调用采样定时器 n 次。"""
    assert fake.timers, '节点未创建定时器'
    _period, cb = fake.timers[0]
    for _ in range(n):
        clock.advance(dt)
        cb()


def _send_gt(fake: _FakeNode, point_cls: Any, x: float, y: float,
             z: float = 0.0) -> None:
    """发一条真值（world 系 PointStamped）。"""
    msg = point_cls()
    msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(z)
    fake.subs[fake.gt_topic](msg)


def _send_state(fake: _FakeNode, state_cls: Any, yaw: float, pitch: float) -> None:
    """发一条云台状态（quat = world→gimbal_link）。"""
    from gimbal_common import frames as FR
    q = FR.quat_from_mat(FR.compose_yaw_pitch(yaw, pitch))
    msg = state_cls()
    msg.quat = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
    fake.subs['/gimbal/state'](msg)


def _align(fake: _FakeNode, stub_ros: Any, clock: _Clock,
           radius: float = 2.0) -> None:
    """让云台恰好对准 +x 方向的真值点（方便断言 0° 误差）。"""
    _send_gt(fake, stub_ros.PointStamped, radius, 0.0)
    _send_state(fake, stub_ros.GimbalState, 0.0, 0.0)
    _tick(fake, clock, 1)


# ---------------------------------------------------------------------------
# 1. 接线
# ---------------------------------------------------------------------------
def test_wiring(stub_ros, tmp_path):
    """订阅真值 + 云台状态 + 估计 + map→world；发布误差与轨迹；100Hz 定时器。"""
    _node, app, fake, clock = _boot(stub_ros, tmp_path)
    _align(fake, stub_ros, clock)               # 冒烟：一次正常采样

    assert GT_TOPIC_PARAM in app._params
    assert fake.gt_topic in fake.subs
    assert '/gimbal/state' in fake.subs
    assert '/estimation/target_state' in fake.subs
    assert '/gimbal/t_map_world' in fake.subs
    assert '/debug/angle_error_deg' in fake.pubs
    assert '/debug/target_path' in fake.pubs
    assert len(fake.timers) == 1
    assert fake.timers[0][0] == pytest.approx(1.0 / 100.0)
    assert fake.errors()[-1] == pytest.approx(0.0, abs=1e-9)

    # 半视场由 virtual.yaml 的 width/fx 推出（不硬编码内参）
    from gimbal_common import config_loader as CL
    cam = CL.load_yaml(VIRTUAL_YAML, required=True)['camera']
    expect = math.atan((float(cam['width']) * 0.5) / float(cam['fx']))
    assert app._half_fov == pytest.approx(expect)
    assert 0.0 < app._half_fov < math.pi/2  # use calibrated K, not the former synthetic 60deg FOV


def test_skips_until_data_arrives(stub_ros, tmp_path):
    """数据未就绪时不发布误差；shutdown 仍能落盘且不得判 PASS。"""
    _node, app, fake, clock = _boot(stub_ros, tmp_path)
    _tick(fake, clock, 5)
    assert fake.errors() == [], '无输入时不得发布误差'
    assert app._n_skipped == 5

    app.shutdown()
    text = (tmp_path / 'latest' / 'eval_report.md').read_text(encoding='utf-8')
    assert 'FAIL' in text, '无样本时不得判 PASS'
    assert '未采集到任何有效样本' in text


def test_stale_input_is_skipped(stub_ros, tmp_path):
    """输入超过 `debug.stale_timeout_s` 未更新后停止采样。"""
    _node, app, fake, clock = _boot(stub_ros, tmp_path,
                                    **{'debug.stale_timeout_s': 0.2})
    _send_gt(fake, stub_ros.PointStamped, 2.0, 0.0)
    _send_state(fake, stub_ros.GimbalState, 0.0, 0.0)
    _tick(fake, clock, 1)
    n1 = len(app._samples)
    assert n1 == 1

    # 超时窗口内仍会采样（窗口 0.2s / 步长 0.01s → 最多再采 20 次）
    _tick(fake, clock, 100, dt=0.01)
    n2 = len(app._samples)
    assert n2 > n1, '超时窗口内应继续采样'
    assert n2 <= 1 + 21, '不应超过超时窗口允许的次数'
    assert app._n_skipped > 0

    # 再推进：已超时，必须完全停止采样
    _tick(fake, clock, 10, dt=0.01)
    assert len(app._samples) == n2, '超时后不得继续采样'


# ---------------------------------------------------------------------------
# 2. e_ang 与 PRD §6.1 定义一致
# ---------------------------------------------------------------------------
def test_e_ang_matches_prd_definition(stub_ros, tmp_path):
    """目标在光轴上 → 0°；云台偏 10° → 10°；目标偏 20° → 20°。"""
    _node, _app, fake, clock = _boot(stub_ros, tmp_path)

    _send_gt(fake, stub_ros.PointStamped, 2.0, 0.0)
    _send_state(fake, stub_ros.GimbalState, 0.0, 0.0)
    _tick(fake, clock, 1)
    assert fake.errors()[-1] == pytest.approx(0.0, abs=1e-9)

    _send_state(fake, stub_ros.GimbalState, math.radians(10.0), 0.0)
    _tick(fake, clock, 1)
    assert fake.errors()[-1] == pytest.approx(10.0, abs=1e-9)

    _send_gt(fake, stub_ros.PointStamped,
             2.0 * math.cos(math.radians(20.0)), 2.0 * math.sin(math.radians(20.0)))
    _send_state(fake, stub_ros.GimbalState, 0.0, 0.0)
    _tick(fake, clock, 1)
    assert fake.errors()[-1] == pytest.approx(20.0, abs=1e-9)


def test_pitch_error_matches_prd_definition(stub_ros, tmp_path):
    """俯仰方向同样按 world 系夹角计算：目标抬高 15° → 15°。"""
    _node, _app, fake, clock = _boot(stub_ros, tmp_path)
    a = math.radians(15.0)
    _send_gt(fake, stub_ros.PointStamped, 2.0 * math.cos(a), 0.0, 2.0 * math.sin(a))
    _send_state(fake, stub_ros.GimbalState, 0.0, 0.0)
    _tick(fake, clock, 1)
    assert fake.errors()[-1] == pytest.approx(15.0, abs=1e-9)


def test_gimbal_origin_offset_is_honoured(stub_ros, tmp_path):
    """`debug.gimbal_origin_xyz` 生效：误差按「目标相对云台原点」计算。"""
    _node, _app, fake, clock = _boot(stub_ros, tmp_path,
                                     **{'debug.gimbal_origin_xyz': [10.0, 0.0, 0.0]})
    _send_state(fake, stub_ros.GimbalState, math.pi / 2.0, 0.0)   # 光轴指向 +y
    _send_gt(fake, stub_ros.PointStamped, 10.0, 2.0, 0.0)         # 原点 +y 方向 2m
    _tick(fake, clock, 1)
    assert fake.errors()[-1] == pytest.approx(0.0, abs=1e-6)

    _send_gt(fake, stub_ros.PointStamped, 15.0, 0.0, 0.0)         # 原点 +x 方向 5m
    _tick(fake, clock, 1)
    assert fake.errors()[-1] == pytest.approx(90.0, abs=1e-6)


def test_nonfinite_gt_is_not_published(stub_ros, tmp_path):
    """真值非有限 → 不发布误差，且该样本被统计过滤。"""
    import gimbal_debug.metrics as M

    _node, app, fake, clock = _boot(stub_ros, tmp_path)
    _send_state(fake, stub_ros.GimbalState, 0.0, 0.0)
    _send_gt(fake, stub_ros.PointStamped, float('nan'), 0.0)
    _tick(fake, clock, 1)
    assert fake.errors() == []
    assert len(app._samples) == 1                       # 已采样
    assert M.values_from_samples(app._samples) == ([], [])   # 但被统计剔除


# ---------------------------------------------------------------------------
# 3. 轨迹发布
# ---------------------------------------------------------------------------
def test_publishes_ground_truth_path(stub_ros, tmp_path):
    """按 `debug.path_period_s` 周期发布真值轨迹，frame_id = world。"""
    _node, _app, fake, clock = _boot(stub_ros, tmp_path,
                                     **{'debug.path_period_s': 0.1})
    for k in range(30):
        _send_gt(fake, stub_ros.PointStamped, 2.0, 0.1 * k)
        _send_state(fake, stub_ros.GimbalState, 0.0, 0.0)
        _tick(fake, clock, 1)
    paths = fake.paths()
    assert paths, '应有轨迹发布'
    assert paths[-1].header.frame_id == 'world'
    assert len(paths[-1].poses) > 0
    assert paths[-1].poses[-1].pose.position.x == pytest.approx(2.0)


def test_path_point_budget_is_enforced(stub_ros, tmp_path):
    """`debug.path_max_points` 生效；未到发布周期不发布。"""
    _node, app, fake, clock = _boot(stub_ros, tmp_path,
                                    **{'debug.path_period_s': 1e9,
                                       'debug.path_max_points': 10})
    for _ in range(40):
        _send_gt(fake, stub_ros.PointStamped, 2.0, 0.0)
        _send_state(fake, stub_ros.GimbalState, 0.0, 0.0)
        _tick(fake, clock, 1)
    assert fake.paths() == [], '周期未到不应发布'
    assert len(app._path.poses) <= 10, '内部轨迹缓存必须被裁剪'


def test_30s_run_writes_artifacts_and_passes(stub_ros, tmp_path):
    """30s@100Hz，收敛后 0.6° → 报告 PASS、数值正确、三件产物齐全。"""
    _node, app, fake, clock = _boot(stub_ros, tmp_path,
                                    **{'debug.sample_rate_hz': 100.0})
    _run_30s(fake, clock, stub_ros, err_deg=0.6, first_s=0.8, first_err_deg=12.0,
             swing_deg=20.0, n=3000)

    errs = fake.errors()
    assert len(errs) == 3000
    assert errs[0] == pytest.approx(12.0, abs=1e-6)
    assert errs[-1] == pytest.approx(0.6, abs=1e-6)

    app.shutdown()
    run_dir = tmp_path / 'latest'
    report, curve, metrics = (run_dir / 'eval_report.md',
                              run_dir / 'eval_error_curve.csv',
                              run_dir / 'eval_metrics.json')
    for p in (report, curve, metrics):
        assert p.is_file(), '缺少产物 %s' % p.name

    md = report.read_text(encoding='utf-8')
    for token in ('角度 RMS', '角度 max', '收敛时间', 'world 系'):
        assert token in md
    assert 'PASS' in md

    mj = json.loads(metrics.read_text(encoding='utf-8'))
    assert mj['n_samples'] == 3000
    assert mj['rms_deg'] == pytest.approx(0.6, abs=0.02), '前段 12° 应被统计窗口排除'
    assert mj['max_deg'] == pytest.approx(0.6, abs=0.02)
    assert mj['convergence_time_s'] == pytest.approx(0.8, abs=0.02)
    assert mj['passed'] is True
    assert mj['valid'] is True

    lines = curve.read_text(encoding='utf-8').splitlines()
    assert len(lines) == 3000 + 1
    assert lines[0].startswith('# t_mono,t_rel_s,e_ang_deg')


def test_30s_run_marks_fail_on_large_error(stub_ros, tmp_path):
    """全程 9° 误差 → 报告 FAIL，且不达标项被标记。"""
    _node, app, fake, clock = _boot(stub_ros, tmp_path,
                                    **{'debug.sample_rate_hz': 100.0})
    _run_30s(fake, clock, stub_ros, err_deg=9.0, n=1200)
    app.shutdown()
    md = (tmp_path / 'latest' / 'eval_report.md').read_text(encoding='utf-8')
    assert 'FAIL' in md
    assert '❌' in md
    mj = json.loads((tmp_path / 'latest' / 'eval_metrics.json').read_text('utf-8'))
    assert mj['passed'] is False
    assert mj['rms_deg'] > 2.0
    assert mj['max_deg'] > 5.0


def test_swing_check_can_invalidate_run(stub_ros, tmp_path):
    """`require_swing=true` 且摆幅 < 半视场 → invalid（hardware_spec §5）。"""
    _node, app, fake, clock = _boot(stub_ros, tmp_path,
                                    **{'debug.sample_rate_hz': 100.0,
                                       'debug.require_swing': True})
    _run_30s(fake, clock, stub_ros, err_deg=0.5, swing_deg=3.0, n=1200)
    app.shutdown()
    mj = json.loads((tmp_path / 'latest' / 'eval_metrics.json').read_text('utf-8'))
    assert mj['valid'] is False
    assert mj['passed'] is False
    assert '摆幅' in mj['reason']
    assert mj['swing_deg'] < 30.0                      # 半视场约 30°


def test_rms_window_skip_is_configurable(stub_ros, tmp_path):
    """`debug.post_convergence_skip_s` 控制 RMS 统计窗口起点。"""
    _node, app, fake, clock = _boot(stub_ros, tmp_path,
                                    **{'debug.sample_rate_hz': 100.0,
                                       'debug.post_convergence_skip_s': 5.0})
    _run_30s(fake, clock, stub_ros, err_deg=0.6, first_s=4.0, first_err_deg=10.0,
             n=1500)
    app.shutdown()
    mj = json.loads((tmp_path / 'latest' / 'eval_metrics.json').read_text('utf-8'))
    assert mj['rms_deg'] == pytest.approx(0.6, abs=0.02), '4s 的大误差应被 5s 窗口排除'
    assert mj['max_deg'] == pytest.approx(0.6, abs=0.02)


def test_estimate_vs_truth_stats(stub_ros, tmp_path):
    """收到 map→world 后附加统计生效；未收到时报告中显示 n/a。"""
    # (a) 未收到 map→world
    _n1, app1, fake1, clock1 = _boot(stub_ros, tmp_path,
                                     **{'debug.sample_rate_hz': 100.0})
    for _ in range(20):
        _send_gt(fake1, stub_ros.PointStamped, 2.0, 0.0)
        _send_state(fake1, stub_ros.GimbalState, 0.0, 0.0)
        _tick(fake1, clock1, 1)
    app1.shutdown()
    md = (tmp_path / 'latest' / 'eval_report.md').read_text(encoding='utf-8')
    assert '| 估计 vs 真值 平均距离(m) | n/a |' in md

    # (b) 收到 map→world（单位阵）+ 估计（偏心 2cm）→ 有数值
    _n2, app2, fake2, clock2 = _boot(stub_ros, tmp_path,
                                     **{'debug.sample_rate_hz': 100.0})
    tf = stub_ros.TransformStamped()
    tf.transform.rotation.w = 1.0
    fake2.subs['/gimbal/t_map_world'](tf)
    est = stub_ros.TargetState()
    est.valid = True
    est.track_status = 1
    est.cur_x, est.cur_y = 2.0 - 0.02, 0.0
    for _ in range(20):
        _send_gt(fake2, stub_ros.PointStamped, 2.0, 0.0)
        _send_state(fake2, stub_ros.GimbalState, 0.0, 0.0)
        fake2.subs['/estimation/target_state'](est)
        _tick(fake2, clock2, 1)
    app2.shutdown()
    mj = json.loads((tmp_path / 'latest' / 'eval_metrics.json').read_text('utf-8'))
    assert mj['est_err_mean_m'] == pytest.approx(0.02, rel=1e-6)
    assert mj['track_status_hist'] == {'1': 20}


# ---------------------------------------------------------------------------
# 30s 跑批
# ---------------------------------------------------------------------------
def _run_30s(fake: _FakeNode, clock: _Clock, stub_ros: Any, *,
             err_deg: float, n: int = 3000, first_s: float = 0.0,
             first_err_deg: float = 0.0, swing_deg: float = 20.0) -> None:
    """跑 n 步（步长 0.01s）：目标做正弦摆动，云台带指定误差跟踪。

    Args:
        err_deg: 稳态跟踪误差（度）。
        first_s: 前 first_s 秒用 first_err_deg 的大误差（模拟收敛前）。
        swing_deg: 目标摆幅（度）。
    """
    from gimbal_common import frames as FR

    cb_gt = fake.subs[fake.gt_topic]
    cb_st = fake.subs['/gimbal/state']
    _period, tick = fake.timers[0]
    for k in range(n):
        t_rel = k * 0.01
        ang = math.radians(swing_deg * math.sin(2.0 * math.pi * 0.05 * t_rel))
        m = stub_ros.PointStamped()
        m.point.x, m.point.y, m.point.z = 2.0 * math.cos(ang), 2.0 * math.sin(ang), 0.0
        cb_gt(m)
        e_deg = first_err_deg if t_rel < first_s else err_deg
        q = FR.quat_from_mat(FR.compose_yaw_pitch(ang + math.radians(e_deg), 0.0))
        s = stub_ros.GimbalState()
        s.quat = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        cb_st(s)
        clock.advance(0.01)
        tick()
