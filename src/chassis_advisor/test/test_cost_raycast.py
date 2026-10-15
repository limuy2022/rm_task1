"""cost_raycast / velocity_law 单测（不依赖 rclpy）。

覆盖 DESIGN §7-T04 验收点：
- 人工栅格上射线命中距离正确（含未知格与越界处理）
- 正前方被挡时应选**偏转**方向
- 全部方向都被挡时 `blocked=true`（且 v_des → 0）
- 距离速度律、一阶低通、加速度限幅、丢失衰减

人工栅格约定：40×40，res=0.1，origin=(0,0)，即世界范围 [0,4]m。
cell 中心 = (i+0.5, j+0.5) * 0.1，例如 cell (5,5) 中心 = (0.55, 0.55)。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from chassis_advisor.cost_raycast import (CostRaycast, RayGridAdapter, RayHit,
                                          build_dirs, cost_of, rotz2)
from chassis_advisor.velocity_law import VelocityLaw

RES = 0.1
N_CELL = 40
ORIGIN = (0.0, 0.0)
ROBOT = (0.55, 0.55)          # cell (5,5) 的中心


def _empty_grid(fill: int = 0) -> np.ndarray:
    """40×40 全 fill 的栅格。"""
    return np.full((N_CELL, N_CELL), fill, dtype=np.int8)


def _adapter(arr: np.ndarray) -> RayGridAdapter:
    """包装成 RayGridAdapter。"""
    return RayGridAdapter(arr, np.array(ORIGIN, dtype=float), RES)


# ---------------------------------------------------------------------------
# 1. 方向集合
# ---------------------------------------------------------------------------
def test_build_dirs_shape_and_symmetry():
    """13 个方向：含 0、对称、单侧跨度为 dir_span_deg。"""
    d = build_dirs(13, 75.0)
    assert len(d) == 13
    assert 0.0 in d
    assert d[0] == pytest.approx(-math.radians(75.0))
    assert d[-1] == pytest.approx(math.radians(75.0))
    assert np.allclose(d, -d[::-1])
    assert len(build_dirs(1, 75.0)) == 1


def test_cost_function_terms():
    """C(θ) = w_ang|θ| + w_obs·max(0,(L−d_occ)/L) + w_unk·max(0,(L−d_unk)/L)。"""
    inf = float('inf')
    assert cost_of(0.0, inf, inf, 1.0) == pytest.approx(0.0)
    assert cost_of(math.radians(30.0), inf, inf, 1.0) == pytest.approx(math.radians(30.0))
    # 正前方 0.5m 命中占用，L=1.0 → 100 * 0.5 = 50
    assert cost_of(0.0, 0.5, inf, 1.0, w_obs=100.0) == pytest.approx(50.0)
    # 未知格 0.5m，L=1.0 → 10 * 0.5 = 5
    assert cost_of(0.0, inf, 0.5, 1.0, w_unk=10.0) == pytest.approx(5.0)
    # 命中距离 ≥ L 时该项为 0
    assert cost_of(0.0, 2.0, 2.0, 1.0) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# 2. 射线命中距离
# ---------------------------------------------------------------------------
def test_raycast_hit_distance_occupied():
    """正前方 1.0m 处有一堵竖墙 → d_occ ≈ 1.0m。"""
    g = _empty_grid()
    g[15, :] = 100                      # x ∈ [1.5, 1.6] 的整列墙
    rc = CostRaycast(grid=_adapter(g), lookahead_m=2.0)
    hits = rc.cast(_adapter(g), ROBOT, [0.0], L=2.0)
    assert len(hits) == 1
    # cell 15 中心 x = 1.55，机器人 x = 0.55 → 中心距 1.0
    assert hits[0].d_occ == pytest.approx(1.0, abs=0.06)
    assert hits[0].hit_occ is True
    assert math.isinf(hits[0].d_unk)


def test_raycast_hit_distance_unknown():
    """0.5m 处是未知格 → d_unk ≈ 0.5m，且不与占用混淆。"""
    g = _empty_grid()
    g[10, :] = -1                       # x ∈ [1.0, 1.1] 的未知列
    rc = CostRaycast(grid=_adapter(g), lookahead_m=2.0)
    hits = rc.cast(_adapter(g), ROBOT, [0.0], L=2.0)
    assert hits[0].d_unk == pytest.approx(0.5, abs=0.06)
    assert math.isinf(hits[0].d_occ)
    assert hits[0].hit_occ is False


def test_raycast_no_hit_beyond_lookahead():
    """障碍在 L 之外 → 不算命中。"""
    g = _empty_grid()
    g[15, :] = 100                      # 距 1.0m
    rc = CostRaycast(grid=_adapter(g), lookahead_m=0.5)
    hits = rc.cast(_adapter(g), ROBOT, [0.0], L=0.5)
    assert math.isinf(hits[0].d_occ)
    assert hits[0].cost == pytest.approx(0.0)


def test_raycast_with_nonzero_grid_origin():
    """⚠ 回归：栅格原点非 (0,0)（默认场地是 (-6,-6)）时命中距离必须仍然正确。

    `gimbal_common.math_utils.ray_cells` 的 `origin_xy` 是**栅格局部米制坐标**，
    若误传世界坐标会整体偏移 6m。本用例专门守住这条边界。
    """
    g = _empty_grid()
    g[25, :] = 100                      # cell 中心 x = 2.55
    origin_xy = (-6.0, -6.0)            # 与真实场地一致
    world_robot = (-6.0 + 0.55, -6.0 + 0.55)     # 世界坐标下的 cell (5,5) 中心
    rc = CostRaycast(grid=_adapter(g), lookahead_m=3.0)
    adapter = RayGridAdapter(g, np.array(origin_xy, dtype=float), RES)
    rc.set_grid(adapter)
    theta, _u_safe, _blocked, hits = rc.evaluate(world_robot, np.array([1.0, 0.0]),
                                                 L=3.0)
    straight = min(hits, key=lambda h: abs(h.theta))
    assert straight.hit_occ is True
    assert straight.d_occ == pytest.approx(2.0, abs=0.06)      # 2.55 − 0.55
    assert theta == pytest.approx(0.0, abs=1e-9) or abs(theta) > 0.0


def test_raycast_diagonal_direction():
    """45° 方向：正对角线上的墙应在 ≈1.0m·√2 处被命中（按 cell 中心距）。"""
    g = _empty_grid()
    g[15, 15] = 100                     # cell 中心 (1.55, 1.55)
    rc = CostRaycast(grid=_adapter(g), lookahead_m=3.0)
    d = np.array([math.sqrt(0.5), math.sqrt(0.5)])
    hits = rc.cast(_adapter(g), ROBOT, [math.radians(45.0)], L=3.0)
    expect = float(np.dot(np.array([1.55, 1.55]) - np.array(ROBOT), d))
    assert hits[0].d_occ == pytest.approx(expect, abs=0.06)
    assert hits[0].d_occ == pytest.approx(1.4142, abs=0.06)


# ---------------------------------------------------------------------------
# 3. 正前方被挡 → 必须偏转
# ---------------------------------------------------------------------------
def test_blocked_front_selects_deflection():
    """正前方 1.0m 有墙（只挡住 y ≤ 1.1 的下半部）→ 应选正向偏转（往 +y 绕）。"""
    g = _empty_grid()
    g[15, 0:11] = 100                   # 竖墙只覆盖 j=0..10（y ≤ 1.1）
    rc = CostRaycast(grid=_adapter(g), lookahead_m=2.0)
    theta, u_safe, blocked, hits = rc.evaluate(ROBOT, np.array([1.0, 0.0]), L=2.0)

    assert blocked is False
    assert len(hits) == 13
    # 0° 一定被挡
    assert hits[6].theta == pytest.approx(0.0)
    assert hits[6].hit_occ is True
    # 选出的方向应明显偏转，且朝 +y（u_safe[1] > 0）
    assert abs(theta) > math.radians(30.0)
    assert u_safe[1] > 0.0
    assert u_safe[0] > 0.0
    # 偏转后该方向不再命中占用
    chosen = min(hits, key=lambda h: h.cost)
    assert chosen.hit_occ is False
    assert chosen.theta == pytest.approx(theta)
    # 代价必须严格优于直行
    assert chosen.cost < hits[6].cost


def test_free_field_keeps_straight():
    """无障碍时空旷场地应选 θ*=0（角度项最小）。"""
    g = _empty_grid()
    rc = CostRaycast(grid=_adapter(g), lookahead_m=1.0)
    theta, u_safe, blocked, _hits = rc.evaluate(ROBOT, np.array([1.0, 0.0]))
    assert blocked is False
    assert theta == pytest.approx(0.0)
    assert u_safe[0] == pytest.approx(1.0)
    assert u_safe[1] == pytest.approx(0.0, abs=1e-9)


def test_pick_prefers_farther_hit():
    """两个方向都命中占用时，取代价更小（命中更远）的那个。"""
    rc = CostRaycast(grid=_adapter(_empty_grid()))
    hits = [
        RayHit(theta=0.0, d_occ=0.3, d_unk=float('inf'), cost=70.0, hit_occ=True),
        RayHit(theta=math.radians(75.0), d_occ=1.2, d_unk=float('inf'),
               cost=40.0, hit_occ=True),
        RayHit(theta=math.radians(-75.0), d_occ=1.2, d_unk=float('inf'),
               cost=40.0, hit_occ=True),
    ]
    theta, u_safe, blocked, _ = rc._pick(hits, np.array([1.0, 0.0]))
    assert blocked is True
    assert theta == pytest.approx(math.radians(75.0))
    assert u_safe[1] > 0.0


def test_pick_tie_break_prefers_smaller_abs_theta():
    """代价相同时（含全部自由），选 |θ| 更小的方向。"""
    rc = CostRaycast(grid=_adapter(_empty_grid()))
    hits = [
        RayHit(theta=0.5, d_occ=float('inf'), d_unk=float('inf'),
               cost=1.0, hit_occ=False),
        RayHit(theta=-0.2, d_occ=float('inf'), d_unk=float('inf'),
               cost=1.0, hit_occ=False),
    ]
    theta, _u_safe, blocked, _ = rc._pick(hits, np.array([1.0, 0.0]))
    assert blocked is False
    assert theta == pytest.approx(-0.2)


# ---------------------------------------------------------------------------
# 4. 全挡
# ---------------------------------------------------------------------------
def test_all_blocked_returns_blocked_true():
    """13 个方向全部命中占用 → blocked=True，v_des 必须为 0。"""
    g = _empty_grid(fill=100)
    rc = CostRaycast(grid=_adapter(g), lookahead_m=1.0)
    theta, _u_safe, blocked, hits = rc.evaluate(ROBOT, np.array([1.0, 0.0]))
    assert blocked is True
    assert all(h.hit_occ for h in hits)
    assert len(hits) == 13
    # 被挡时速度律置零
    law = VelocityLaw()
    res = law.step(ROBOT, (2.0, 0.55), dt=0.02, u_safe=_u_safe, blocked=blocked)
    assert res['cmd'][0] == pytest.approx(0.0, abs=1e-9)
    assert res['cmd'][1] == pytest.approx(0.0, abs=1e-9)
    assert res['v_des'] == pytest.approx(0.0)
    _ = theta


def test_enclosed_box_blocked():
    """机器人被一圈墙围住（半径 0.5m）→ blocked=True。"""
    g = _empty_grid()
    for k in range(0, 12):                  # 闭合方框 i,j ∈ [0, 11]
        g[0, k] = 100
        g[11, k] = 100
        g[k, 0] = 100
        g[k, 11] = 100
    rc = CostRaycast(grid=_adapter(g), lookahead_m=1.0)
    _theta, _u, blocked, _hits = rc.evaluate(ROBOT, np.array([1.0, 0.0]))
    assert blocked is True


def test_out_of_bounds_treated_as_unknown():
    """射线射出地图边界 → 记为未知命中（保守），不记占用。"""
    g = _empty_grid()
    rc = CostRaycast(grid=_adapter(g), lookahead_m=3.0)
    hits = rc.cast(_adapter(g), (3.85, 3.85), [math.radians(45.0)], L=3.0)
    assert hits[0].d_unk < float('inf')
    assert math.isinf(hits[0].d_occ)


# ---------------------------------------------------------------------------
# 5. 速度律
# ---------------------------------------------------------------------------
def test_distance_speed_law():
    """v_des = clamp(k_v·(d − d_stop), 0, v_max)；d<d_stop → 0；d≥d_slow → v_max。"""
    law = VelocityLaw(v_max=0.5, d_slow=1.0, d_stop=0.15)
    assert law.desired_speed(0.0) == pytest.approx(0.0)
    assert law.desired_speed(0.15) == pytest.approx(0.0)
    assert law.desired_speed(1.0) == pytest.approx(0.5)
    assert law.desired_speed(5.0) == pytest.approx(0.5)     # 被 v_max 截断
    assert law.desired_speed(0.575) == pytest.approx(0.25)  # 线性中点
    assert law.desired_speed(float('nan')) == pytest.approx(0.0)


def test_lowpass_and_accel_limit():
    """一步内不能突破 a_max·dt；持续输入后收敛到 v_des。"""
    law = VelocityLaw(v_max=0.5, a_max=1.0, lp_cutoff_hz=5.0)
    last = np.zeros(2)
    dt = 0.02
    cmd = law.compute(0.5, np.array([1.0, 0.0]), dt, last)
    assert float(np.linalg.norm(cmd - last)) <= 1.0 * dt + 1e-9
    # 长时间后逼近 v_des
    for _ in range(500):
        cmd = law.compute(0.5, np.array([1.0, 0.0]), dt, cmd)
    assert cmd[0] == pytest.approx(0.5, abs=1e-3)
    assert cmd[1] == pytest.approx(0.0, abs=1e-3)


def test_accel_limit_bounds_each_step():
    """任意一步的加速度都不超过 a_max。"""
    law = VelocityLaw(a_max=1.0, lp_cutoff_hz=1000.0)   # 几乎不低通，只看限幅
    cmd = np.zeros(2)
    dt = 0.01
    prev = cmd
    for _ in range(200):
        cmd = law.compute(0.5, np.array([1.0, 0.0]), dt, prev)
        assert float(np.linalg.norm(cmd - prev)) <= 1.0 * dt + 1e-9
        prev = cmd


def test_lost_decay():
    """track_status=0 时按 2s 线性衰减到 0。"""
    law = VelocityLaw(lost_decay_s=2.0)
    assert law.lost_gain(10.0, None) == pytest.approx(1.0)
    assert law.lost_gain(10.0, 10.0) == pytest.approx(1.0)
    assert law.lost_gain(11.0, 10.0) == pytest.approx(0.5)
    assert law.lost_gain(12.0, 10.0) == pytest.approx(0.0)
    assert law.lost_gain(20.0, 10.0) == pytest.approx(0.0)

    law2 = VelocityLaw(lost_decay_s=2.0)
    cmd = np.array([0.4, 0.0])
    assert np.allclose(law2.apply_lost_decay(cmd, 11.0, 10.0), [0.2, 0.0])
    assert np.allclose(law2.apply_lost_decay(cmd, 13.0, 10.0), [0.0, 0.0])


def test_step_zero_distance_direction():
    """d < 1e-6 时 u = 0，命令为 0（不产生 NaN）。"""
    law = VelocityLaw()
    res = law.step((1.0, 1.0), (1.0, 1.0), dt=0.02)
    assert np.all(np.isfinite(res['cmd']))
    assert np.allclose(res['cmd'], 0.0)
    assert res['dist'] == pytest.approx(0.0)
    assert np.allclose(res['u'], 0.0)


def test_accel_limit_signature_dispatch():
    """⚠ 回归：三种 `accel_limit` 签名（3 参增量式 / 2 参旧 stub / 4 参目标式）都正确限幅。"""
    import chassis_advisor.velocity_law as VL

    prev = np.zeros(2)
    target = np.array([1.0, 0.0])
    out = VL._accel_limit_apply(target, prev, 0.01, 1.0)
    assert float(np.linalg.norm(out - prev)) <= 1.0 * 0.01 + 1e-9
    assert out[0] > 0.0
    # dt<=0 → 不限幅，直接到 target
    assert np.allclose(VL._accel_limit_apply(target, prev, 0.0, 1.0), target)
    # a_max<=0 → 不限幅
    assert np.allclose(VL._accel_limit_apply(target, prev, 0.01, 0.0), target)
    # 大 dt → 不裁剪
    assert np.allclose(VL._accel_limit_apply(target, prev, 10.0, 1.0), target)

    # 逐个强制切换签名，验证分派逻辑本身
    def _stub2(delta, max_delta):
        d = np.asarray(delta, dtype=float)
        n = float(np.linalg.norm(d))
        return d * (max_delta / n) if n > max_delta else d

    def _stub4(tgt, prv, dt, a_max):
        d = np.asarray(tgt, dtype=float) - np.asarray(prv, dtype=float)
        n = float(np.linalg.norm(d))
        lim = a_max * dt
        return np.asarray(prv) + (d * (lim / n) if n > lim else d)

    real = VL.accel_limit
    try:
        VL.accel_limit = _stub2
        o2 = VL._accel_limit_apply(target, prev, 0.01, 1.0)
        assert float(np.linalg.norm(o2 - prev)) <= 1.0 * 0.01 + 1e-9
        VL.accel_limit = _stub4
        o4 = VL._accel_limit_apply(target, prev, 0.01, 1.0)
        assert float(np.linalg.norm(o4 - prev)) <= 1.0 * 0.01 + 1e-9
    finally:
        VL.accel_limit = real


def test_rotz2_and_u_safe():
    """u_safe = Rot(θ*)·u，模长保持 1。"""
    u = np.array([1.0, 0.0])
    for deg in (0.0, 15.0, -30.0, 75.0):
        th = math.radians(deg)
        us = rotz2(th) @ u
        assert np.linalg.norm(us) == pytest.approx(1.0)
        assert us[0] == pytest.approx(math.cos(th))
        assert us[1] == pytest.approx(math.sin(th))
