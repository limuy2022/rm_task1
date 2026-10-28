"""方向项解析求解：找"沿地图可行、与目标方向夹角最小"的连续角度。

官方规则（campus-contest-env「计分概要」赛题一）：

> `direction` 权重 **0.2**，误差换算成**度**后除以 180。
> 标准方向是**从底盘出发、沿地图可行的最近方向**（射线最长取 **1 m**）。
> `hypot(vx, vy) < 0.02` 视为停车，此时离目标不到 0.5 m 记 0 分，否则记 1 分。

换算：1° 误差 → `0.2 / 180 ≈ 0.00111` 的总分。方向项 RMSE 从 3.6° 降到 0.5°
相当于总分降 **0.0034**，与其他项同量级，是**性价比最高**的改造点。

判据：射线在 `L = 1.0 m` 内**不与不可跨越地形（占用格 100）相交**。
未知格（-1）默认**不**阻挡——评分系统的标准方向是在它自己的地图上算的，
那里没有未知区；把未知当障碍会系统性偏离标准方向。
可用 `block_unknown=True` 切换成保守模式（实际行驶更安全，但会偏离评分标准）。

算法（不必密集扫角，单次 O(邻域占用格数)）：

1. 取底盘周围 `L + √2·res` 内的所有占用格（1 m @ 0.05 m/cell → 约 400 格）；
2. 每格算出它在底盘处张开的**角度区间** `[a_lo, a_hi]`（凸四边形的角度极值必在顶点取到，
   所以直接取 4 个角点的极值即可，**精确**，无采样误差）；
3. 求区间的并集 → 被阻挡的角域；取补集 → 可行角域；
4. 在可行角域中取**离目标方向最近**的角度 → 连续的 `θ*`（不受量化步长限制）；
5. 用真实射线投射校验 `θ*`，若因"部分格超出 L"导致的过阻塞，就在 `±4°`
   窗口内以 0.05° 步长回扫，找回真正的最小可行角。

纯 Python + numpy，不依赖 rclpy，可直接单测。
"""

from __future__ import annotations

import math
from typing import Any
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np

from team43_common.math_utils import ray_cells, wrap_pi

__all__ = [
    'FREE_DIR_L_M',
    'REFINE_WINDOW_DEG',
    'REFINE_STEP_DEG',
    'V_STOP_OFFICIAL',
    'STOP_DIST_OFFICIAL',
    'OCCUPIED',
    'UNKNOWN',
    'blocked_intervals',
    'min_angle_free_dir',
    'FreeDirSolver',
]

#: 官方规定的射线最大长度（m）
FREE_DIR_L_M: float = 1.0
#: 回扫窗口半宽（度）。过阻塞量 ≈ 1 个 cell 在 L 处的张角（res/L→2.9°），
#: 取 8° 留足余量；最坏 160 次射线投射，10 Hz 下约占 1.5% CPU。
REFINE_WINDOW_DEG: float = 8.0
#: 回扫步长（度）
REFINE_STEP_DEG: float = 0.05
#: 官方"视为停车"的速度阈值（m/s）：`hypot(vx,vy) < 0.02`
V_STOP_OFFICIAL: float = 0.02
#: 官方停车判定配套的目标距离阈值（m）：停车时离目标 <0.5 m 记 0 分，否则记 1 分
STOP_DIST_OFFICIAL: float = 0.5

OCCUPIED: int = 100
UNKNOWN: int = -1

_TWO_PI: float = 2.0 * math.pi
_INF: float = float('inf')


# ---------------------------------------------------------------------------
# 几何小工具
# ---------------------------------------------------------------------------
def _aabb_dist(x0: float, y0: float, x1: float, y1: float,
               px: float, py: float) -> float:
    """点 (px,py) 到轴对齐矩形 [x0,x1]×[y0,y1] 的最短距离（点在内部时为 0）。"""
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def _cell_angle_interval(x0: float, y0: float, x1: float, y1: float,
                         px: float, py: float) -> Tuple[float, float]:
    """矩形在观测点处张开的角度区间（**精确**，极值在顶点取到）。

    Returns:
        tuple：(a_lo, a_hi)，弧度，未归一化，宽度 ≤ π。
    """
    cx = 0.5 * (x0 + x1)
    cy = 0.5 * (y0 + y1)
    a_c = math.atan2(cy - py, cx - px)
    das: List[float] = []
    for (x, y) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        das.append(wrap_pi(math.atan2(y - py, x - px) - a_c))
    return a_c + min(das), a_c + max(das)


def _merge_intervals(intervals: List[Tuple[float, float]]
                     ) -> List[Tuple[float, float]]:
    """合并重叠区间（输入按 a_lo 升序）。"""
    if not intervals:
        return []
    out: List[Tuple[float, float]] = [intervals[0]]
    for lo, hi in intervals[1:]:
        if lo <= out[-1][1] + 1e-12:
            if hi > out[-1][1]:
                out[-1] = (out[-1][0], hi)
        else:
            out.append((lo, hi))
    return out


# ---------------------------------------------------------------------------
# 主函数
# ---------------------------------------------------------------------------
def blocked_intervals(grid: Any,
                      origin_xy: Sequence[float],
                      resolution: float,
                      p: Sequence[float],
                      L: float = FREE_DIR_L_M,
                      block_unknown: bool = False) -> Tuple[bool, List[Tuple[float, float]]]:
    """计算被不可跨越地形阻挡的角度区间（**绝对角**域，弧度）。

    Args:
        grid: (W, H) int8 三态栅格，**未膨胀**（`GridMap.grid` 而非 `ray_grid`）。
        origin_xy: 栅格原点 (x, y)，m。
        resolution: m/cell。
        p: 射线起点（底盘位置，m，世界系）。
        L: 射线长度（m）。
        block_unknown: 是否把未知格也算作阻挡（默认 False）。

    Returns:
        tuple：(all_blocked, intervals)
          - all_blocked: True 表示底盘就落在占用格里，360° 全被挡。
          - intervals: 按 a_lo 升序的阻挡区间列表（可能跨越 ±π）。
    """
    g = np.asarray(grid)
    if g.ndim != 2:
        return False, []
    res = float(resolution)
    if res <= 0.0 or L <= 0.0:
        return False, []
    w, h = int(g.shape[0]), int(g.shape[1])
    ox = float(origin_xy[0])
    oy = float(origin_xy[1])
    px = float(p[0]) - ox
    py = float(p[1]) - oy

    reach = L + math.sqrt(2.0) * res
    i0 = int(math.floor((px - reach) / res))
    i1 = int(math.floor((px + reach) / res))
    j0 = int(math.floor((py - reach) / res))
    j1 = int(math.floor((py + reach) / res))
    if not block_unknown:
        i0 = max(i0, 0)
        j0 = max(j0, 0)
        i1 = min(i1, w - 1)
        j1 = min(j1, h - 1)
    if i1 < i0 or j1 < j0:
        return False, []

    want = (OCCUPIED,) if not block_unknown else (OCCUPIED, UNKNOWN)
    raw: List[Tuple[float, float]] = []
    for i in range(i0, i1 + 1):
        x0 = i * res
        x1 = x0 + res
        col = g[i] if 0 <= i < w else None
        for j in range(j0, j1 + 1):
            v = int(col[j]) if col is not None and 0 <= j < h else UNKNOWN
            if v not in want:
                continue
            y0 = j * res
            y1 = y0 + res
            if _aabb_dist(x0, y0, x1, y1, px, py) > L:
                continue                      # 整格都在射线长度之外
            if px >= x0 and px <= x1 and py >= y0 and py <= y1:
                return True, []               # 底盘在障碍内 → 全挡
            raw.append(_cell_angle_interval(x0, y0, x1, y1, px, py))

    if not raw:
        return False, []
    # 归一化到 [-π, π) 后再排序合并（区间宽度 < π，可安全拆成不跨界的段）
    norm: List[Tuple[float, float]] = []
    for lo, hi in raw:
        lo_n = wrap_pi(lo)
        width = hi - lo
        hi_n = lo_n + width
        if hi_n <= math.pi:
            norm.append((lo_n, hi_n))
        else:                                  # 跨越 +π，拆成两段
            norm.append((lo_n, math.pi))
            norm.append((-math.pi, hi_n - _TWO_PI))
    norm.sort(key=lambda t: t[0])
    return False, _merge_intervals(norm)


def _ray_blocked(g: np.ndarray, origin_xy: Sequence[float], res: float,
                 p: Sequence[float], theta: float, L: float,
                 block_unknown: bool) -> bool:
    """真实射线投射：沿绝对角 theta 走 L 米，是否撞到占用/未知格。"""
    px = float(p[0]) - float(origin_xy[0])
    py = float(p[1]) - float(origin_xy[1])
    d = (math.cos(float(theta)), math.sin(float(theta)))
    cells = ray_cells((px, py), d, L, res)
    w, h = int(g.shape[0]), int(g.shape[1])
    for (i, j) in cells:
        if i < 0 or j < 0 or i >= w or j >= h:
            if block_unknown:
                return True
            continue
        v = int(g[i, j])
        if v == OCCUPIED:
            return True
        if block_unknown and v == UNKNOWN:
            return True
    return False


def min_angle_free_dir(grid: Any,
                       origin_xy: Sequence[float],
                       resolution: float,
                       p: Sequence[float],
                       u_target: Sequence[float],
                       L: float = FREE_DIR_L_M,
                       block_unknown: bool = False,
                       refine: bool = True) -> Tuple[float, bool]:
    """求"沿地图可行、与目标方向夹角最小"的偏转角 θ*（弧度）。

    Args:
        grid: (W, H) 三态栅格（**未膨胀**）。
        origin_xy: 栅格原点 (x, y)。
        resolution: m/cell。
        p: 底盘位置（世界系 m）。
        u_target: 指向目标的**单位**方向（世界系）。
        L: 射线长度（m），官方固定 1.0。
        block_unknown: 未知格是否算阻挡。
        refine: 是否在 ±4° 窗口内做 0.05° 回扫（消除过阻塞）。

    Returns:
        tuple：(theta_star, feasible)
          - theta_star: 相对 `u_target` 的偏转角（rad），正为逆时针。
            取值连续，不再受 12.5° 量化限制。
          - feasible: 是否找到了不相交的方向；False 表示 360° 全被挡
            （此时 theta_star 退化为 0，调用方自行决定）。
    """
    g = np.asarray(grid)
    ut = np.asarray(u_target, dtype=float).reshape(-1)
    if ut.size < 2:
        return 0.0, False
    n = float(np.linalg.norm(ut[:2]))
    if n < 1e-9:
        return 0.0, False
    a_t = math.atan2(float(ut[1]) / n, float(ut[0]) / n)

    all_blocked, blocked = blocked_intervals(g, origin_xy, resolution, p, L,
                                             block_unknown)
    if all_blocked:
        return 0.0, False

    # --- 把阻挡区间搬到"相对目标方向"的偏移空间 o ∈ (-π, π] -----------------
    # 目标：找 |o| 最小且未被阻挡的 o。
    segs: List[Tuple[float, float]] = []
    for lo, hi in blocked:
        r_lo = wrap_pi(lo - a_t)
        width = hi - lo
        if width >= _TWO_PI - 1e-9:
            return 0.0, False
        # 区间在偏移空间中的 [r_lo, r_lo+width]，再取 ±2π 的周期副本
        for shift in (-_TWO_PI, 0.0, _TWO_PI):
            a = r_lo + shift
            b = a + width
            if b <= -math.pi or a >= math.pi:
                continue
            segs.append((max(a, -math.pi), min(b, math.pi)))
    segs.sort(key=lambda t: t[0])
    merged = _merge_intervals(segs)

    # --- 补集 = 可行角域；取离 0 最近的点 -----------------------------------
    free: List[Tuple[float, float]] = []
    cursor = -math.pi
    for lo, hi in merged:
        if lo > cursor + 1e-12:
            free.append((cursor, lo))
        cursor = max(cursor, hi)
    if cursor < math.pi - 1e-12:
        free.append((cursor, math.pi))

    if not free:
        return 0.0, False

    best_o = 0.0
    best_d = _INF
    for lo, hi in free:
        if lo <= 0.0 <= hi:
            cand = 0.0
            d = 0.0
        elif hi < 0.0:
            cand = hi
            d = -hi
        else:
            cand = lo
            d = lo
        if d < best_d:
            best_d = d
            best_o = cand
    if not math.isfinite(best_d):
        return 0.0, False

    # --- 回扫：消除"部分格超出 L"造成的过阻塞 -------------------------------
    if refine and abs(best_o) > 1e-9:
        win = math.radians(REFINE_WINDOW_DEG)
        step = math.radians(REFINE_STEP_DEG)
        sign = -1.0 if best_o < 0.0 else 1.0
        n_max = int(min(math.floor(abs(best_o) / step), math.floor(win / step)))
        # 从**最内侧**（k 最大 → |cand| 最小）往回扫，第一个不相交的角即为所求；
        # 这样只需一次命中即可退出，且保证拿到窗口内 |θ| 最小的可行角。
        for k in range(n_max, 0, -1):
            cand = best_o - sign * (k * step)
            if abs(cand) < 1e-12:
                cand = 0.0
            if not _ray_blocked(g, origin_xy, float(resolution), p,
                                a_t + cand, L, block_unknown):
                best_o = cand
                break

    # Closed occupied cells include their corners. The analytic complement
    # can return a tangent boundary, so validate and nudge into the free side.
    if _ray_blocked(g, origin_xy, float(resolution), p, a_t+best_o, L, block_unknown):
        candidates = [best_o-eps for eps in (1e-6, math.radians(.05))]
        candidates += [best_o+eps for eps in (1e-6, math.radians(.05))]
        candidates = sorted(candidates,key=abs)
        for candidate in candidates:
            if -math.pi <= candidate <= math.pi and not _ray_blocked(
                    g, origin_xy, float(resolution), p, a_t+candidate, L, block_unknown):
                return float(candidate), True
        return 0.0, False
    return float(best_o), True


# ---------------------------------------------------------------------------
# 面向对象封装
# ---------------------------------------------------------------------------
class FreeDirSolver:
    """方向项求解器（持有栅格与参数，逐拍调用 :meth:`solve`）。

    Attributes:
        lookahead_m: 射线长度 L（官方 1.0 m）。
        block_unknown: 未知格是否算阻挡（默认 False）。
        refine: 是否做 ±4° 回扫。
    """

    def __init__(self,
                 lookahead_m: float = FREE_DIR_L_M,
                 block_unknown: bool = False,
                 refine: bool = True) -> None:
        self.lookahead_m = float(lookahead_m)
        self.block_unknown = bool(block_unknown)
        self.refine = bool(refine)
        self._grid: Optional[np.ndarray] = None
        self._origin = np.zeros(2, dtype=float)
        self._res: float = 0.05
        self._origin_yaw: float = 0.0
        self._last_theta: float = 0.0
        self._last_feasible: bool = False

    def set_grid(self, grid: Any, origin_xy: Sequence[float],
                 resolution: float, origin_yaw: float = 0.0) -> None:
        """设置栅格（**未膨胀**三态）与原点/分辨率。"""
        self._grid = np.asarray(grid)
        self._origin = np.array([float(origin_xy[0]), float(origin_xy[1])],
                                dtype=float)
        self._res = float(resolution)
        self._origin_yaw = float(origin_yaw)
        if self._grid.ndim != 2 or not all(math.isfinite(v) for v in (*self._origin, self._res, self._origin_yaw)) or self._res <= 0:
            self._grid = None
            raise ValueError('invalid grid geometry')

    @property
    def has_grid(self) -> bool:
        """是否已设置栅格。"""
        return self._grid is not None

    @property
    def last_theta(self) -> float:
        """上一次求得的偏转角（rad）。"""
        return self._last_theta

    @property
    def last_feasible(self) -> bool:
        """上一次是否找到了可行方向。"""
        return self._last_feasible

    def solve(self, p: Sequence[float], u_target: Sequence[float]
              ) -> Tuple[float, bool]:
        """求偏转角（无栅格时退化返回 0 / feasible=False）。"""
        if self._grid is None:
            self._last_theta = 0.0
            self._last_feasible = False
            return 0.0, False
        c, s = math.cos(self._origin_yaw), math.sin(self._origin_yaw)
        Rinv = np.array([[c, s], [-s, c]])
        local_p = Rinv @ (np.asarray(p, dtype=float)-self._origin)
        local_u = Rinv @ np.asarray(u_target, dtype=float)
        theta, ok = min_angle_free_dir(self._grid, (0.0, 0.0), self._res, local_p,
                                       local_u, self.lookahead_m,
                                       self.block_unknown, self.refine)
        self._last_theta = theta
        self._last_feasible = ok
        return theta, ok

    def solve_unit(self, p: Sequence[float], u_target: Sequence[float]
                   ) -> Tuple[np.ndarray, float, bool]:
        """求可行单位方向。

        Returns:
            tuple：(u_safe(2,), theta, feasible)；`feasible=False` 时
            `u_safe` 退化为目标方向本身。
        """
        uv = np.asarray(u_target, dtype=float).reshape(2)
        n = float(np.linalg.norm(uv))
        uu = uv / n if n > 1e-9 else np.array([1.0, 0.0])
        theta, ok = self.solve(p, uu)
        c = math.cos(theta)
        s = math.sin(theta)
        u_safe = np.array([c * uu[0] - s * uu[1], s * uu[0] + c * uu[1]],
                          dtype=float)
        return u_safe, theta, ok
