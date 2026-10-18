"""13 方向射线投射 + 代价函数（DESIGN §6.10 Step 2）。

纯 Python（不依赖 rclpy），可直接单测。

```
候选 θ ∈ 对称分布在 [−dir_span_deg, +dir_span_deg] 的 n_dirs 个角度（含 0）
沿 u_θ = Rot(θ)·u 在 **ray_grid** 上做射线投射，前瞻 L = lookahead_m（默认 1.0m）
d_hit_occ = 首次命中 100 的距离（未命中 = +inf）
d_hit_unk = 首次命中 -1  的距离（未命中 = +inf）

C(θ) = w_ang * |θ|
     + w_obs * max(0, (L − d_hit_occ) / L)
     + w_unk * max(0, (L − d_hit_unk) / L)
θ* = argmin C(θ);   u_safe = Rot(θ*)·u
```

默认 `w_ang = 1.0`、`w_obs = 100.0`、`w_unk = 10.0`。
**全部 n_dirs 个方向都在 L 内命中占用格** → `blocked=true`，`v_des = 0`。

> ⚠ 文档一致性说明：DESIGN §6.10 文字列举的是 `{0, ±15°, …, ±75°}`（**11** 个角），
> 而 §6.13 参数表给的是 `n_dirs=13` / `dir_span_deg=75`。二者不能同时满足。
> 本实现以**参数表为准**（参数是可配置项，文字是示例）：生成 **n_dirs 个**在
> `[−span, +span]` 上均匀对称、含 0 的角度（13 → 步长 12.5°）。已回报主理人。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np

from gimbal_common.math_utils import ray_cells

__all__ = [
    'N_DIRS',
    'DIR_SPAN_DEG',
    'LOOKAHEAD_M',
    'W_ANG',
    'W_OBS',
    'W_UNK',
    'RayHit',
    'RayGridAdapter',
    'CostRaycast',
    'build_dirs',
    'rotz2',
    'cost_of',
]

# 默认参数（DESIGN §6.13 `chassis.*`）
N_DIRS: int = 13
DIR_SPAN_DEG: float = 75.0
LOOKAHEAD_M: float = 1.0
W_ANG: float = 1.0
W_OBS: float = 100.0
W_UNK: float = 10.0

_INF: float = float('inf')


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------
def rotz2(theta: float) -> np.ndarray:
    """二维绕 +z 旋转矩阵 (2,2)。"""
    c = math.cos(float(theta))
    s = math.sin(float(theta))
    return np.array([[c, -s], [s, c]], dtype=float)


def build_dirs(n_dirs: int = N_DIRS,
               dir_span_deg: float = DIR_SPAN_DEG) -> np.ndarray:
    """生成候选偏转角序列（弧度），对称、含 0、按从小到大排列。

    Args:
        n_dirs: 方向个数（默认 13）。
        dir_span_deg: 单侧最大偏转角（度，默认 75）。

    Returns:
        np.ndarray：长度 n_dirs 的角度数组（rad），范围 [-span, +span]。
    """
    n = int(n_dirs)
    if n < 1:
        n = 1
    span = abs(float(dir_span_deg))
    if n == 1:
        return np.array([0.0], dtype=float)
    if n % 2 == 1:                       # 奇数 → 含 0，两侧各 (n-1)/2 个
        half = (n - 1) // 2
        ks = [k for k in range(-half, half + 1)]
        step = span / float(half)
    else:                                # 偶数 → 对称但不取 0
        half = n // 2
        ks = [k for k in range(-half, half + 1) if k != 0]
        step = span / float(half)
    return np.array([k * step for k in ks], dtype=float) * (math.pi / 180.0)


def cost_of(theta: float, d_occ: float, d_unk: float, L: float,
            w_ang: float = W_ANG, w_obs: float = W_OBS,
            w_unk: float = W_UNK) -> float:
    """代价函数 C(θ)（DESIGN §6.10）。"""
    ll = float(L) if float(L) > 1e-9 else 1e-9
    c_ang = float(w_ang) * abs(float(theta))
    c_obs = float(w_obs) * max(0.0, (ll - float(d_occ)) / ll)
    c_unk = float(w_unk) * max(0.0, (ll - float(d_unk)) / ll)
    return c_ang + c_obs + c_unk


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------
@dataclass
class RayHit:
    """单个方向的投射结果。

    Attributes:
        theta: 偏转角（rad）。
        d_occ: 首次命中占用格的距离（m）；未命中为 +inf。
        d_unk: 首次命未知格的距离（m）；未命中为 +inf。
        cost: 该方向的代价 C(θ)。
        hit_occ: 是否在 L 内命中占用格。
    """

    theta: float = 0.0
    d_occ: float = _INF
    d_unk: float = _INF
    cost: float = 0.0
    hit_occ: bool = False


@dataclass
class RayGridAdapter:
    """把 GridMap / (array, origin, res) 统一成射线投射需要的接口。

    Attributes:
        arr: (W, H) 栅格数组（应为 **ray_grid**，含膨胀）。
        origin_xy: 栅格原点 (x, y)。
        resolution: 分辨率 m/cell。
    """

    arr: np.ndarray
    origin_xy: np.ndarray
    resolution: float
    origin_yaw: float = 0.0

    @property
    def width(self) -> int:
        """栅格宽度（cell）。"""
        return int(self.arr.shape[0])

    @property
    def height(self) -> int:
        """栅格高度（cell）。"""
        return int(self.arr.shape[1])

    def world_to_cell(self, xy: Sequence[float]) -> Tuple[int, int]:
        """世界坐标 → cell 索引。"""
        local = rotz2(-self.origin_yaw) @ (np.asarray(xy, dtype=float)-self.origin_xy)
        i = int(math.floor(local[0] / self.resolution))
        j = int(math.floor(local[1] / self.resolution))
        return (i, j)

    def cell_to_world(self, ij: Sequence[int]) -> np.ndarray:
        """cell 索引 → 中心世界坐标。"""
        return (self.origin_xy + rotz2(self.origin_yaw)
                @ (np.array([int(ij[0]) + 0.5, int(ij[1]) + 0.5], dtype=float)
                   * self.resolution))

    def in_bounds(self, ij: Sequence[int]) -> bool:
        """cell 索引是否在界内。"""
        return bool(0 <= int(ij[0]) < self.width and 0 <= int(ij[1]) < self.height)

    def at(self, ij: Sequence[int]) -> int:
        """取 cell 值；越界视为未知 (-1)（保守）。"""
        if not self.in_bounds(ij):
            return -1
        return int(self.arr[int(ij[0]), int(ij[1])])


def _as_adapter(grid: Any) -> RayGridAdapter:
    """把各种输入归一化成 RayGridAdapter。

    支持：
      1. `grid_mapper.grid_builder.GridMap`（取其 **ray_grid**）
      2. 元组 / 列表 `(arr, origin_xy, resolution)`
      3. 已经是 RayGridAdapter
    """
    if isinstance(grid, RayGridAdapter):
        return grid
    if isinstance(grid, (tuple, list)) and len(grid) >= 3:
        return RayGridAdapter(np.asarray(grid[0]),
                              np.array([float(grid[1][0]), float(grid[1][1])],
                                       dtype=float),
                              float(grid[2]), float(grid[3]) if len(grid) > 3 else 0.0)
    arr = getattr(grid, 'ray_grid', None)
    if arr is None:
        arr = getattr(grid, 'grid', None)
    if arr is None:
        raise TypeError('unsupported grid object: %r' % (type(grid),))
    origin = np.asarray(getattr(grid, 'origin_xy'), dtype=float).reshape(2)
    return RayGridAdapter(np.asarray(arr), origin, float(getattr(grid, 'resolution')),
                          float(getattr(grid, 'origin_yaw', 0.0)))


# ---------------------------------------------------------------------------
# 主类
# ---------------------------------------------------------------------------
class CostRaycast:
    """13 方向射线投射与方向选择。

    Attributes:
        lookahead_m: 前瞻距离 L（m）。
        w_ang / w_obs / w_unk: 代价权重。
        n_dirs / dir_span_deg: 方向集合参数。
        dirs: 候选角度数组（rad）。
    """

    def __init__(self,
                 grid: Any = None,
                 lookahead_m: float = LOOKAHEAD_M,
                 w_ang: float = W_ANG,
                 w_obs: float = W_OBS,
                 w_unk: float = W_UNK,
                 n_dirs: int = N_DIRS,
                 dir_span_deg: float = DIR_SPAN_DEG) -> None:
        self.lookahead_m = float(lookahead_m)
        self.w_ang = float(w_ang)
        self.w_obs = float(w_obs)
        self.w_unk = float(w_unk)
        self.n_dirs = int(n_dirs)
        self.dir_span_deg = float(dir_span_deg)
        self.dirs: np.ndarray = build_dirs(self.n_dirs, self.dir_span_deg)

        self._adapter: Optional[RayGridAdapter] = None
        self._origin = np.zeros(2, dtype=float)
        self._last_hits: List[RayHit] = []
        if grid is not None:
            self.set_grid(grid)

    # -- 配置 -----------------------------------------------------------------
    def set_grid(self, grid: Any) -> None:
        """设置用于投射的栅格（应为含膨胀的 ray_grid）。"""
        self._adapter = _as_adapter(grid)

    def set_origin(self, origin_xy: Sequence[float]) -> None:
        """设置射线起点（底盘位置，map 系）。"""
        self._origin = np.array([float(origin_xy[0]), float(origin_xy[1])],
                                dtype=float)

    @property
    def last_hits(self) -> List[RayHit]:
        """上一次 cast 的结果（便于诊断与单测）。"""
        return self._last_hits

    # -- 投射 -----------------------------------------------------------------
    def cast(self,
             grid: Any,
             origin_xy: Sequence[float],
             dirs: Optional[Sequence[float]] = None,
             L: Optional[float] = None) -> List[RayHit]:
        """对 `dirs` 中每个偏转角做射线投射（**相对基准方向 u=(1,0)**？见说明）。

        说明：`dirs` 是**绝对方向的偏转角序列**；本方法只负责"沿给定世界方向
        `Rot(θ)·u` 投射"。为了让 `choose(u)` 与 `cast()` 解耦，这里把 `dirs`
        解释为**世界系绝对角**（rad），即投射方向 = `[cos θ, sin θ]`。
        当调用方传入 `build_dirs()` 的结果并且 `u` 指向 +x 时二者一致；
        实际使用请以 `evaluate()` / `choose()` 为准（它们内部把 θ 叠加到 u 上）。

        Args:
            grid: GridMap / (arr, origin, res) / RayGridAdapter。
            origin_xy: 射线起点（m）。
            dirs: 候选角度（rad）；None 时用 `self.dirs` 叠加到基准方向。
            L: 前瞻距离；None 时用 `self.lookahead_m`。

        Returns:
            list[RayHit]：与 dirs 等长。
        """
        adapter = _as_adapter(grid)
        self._adapter = adapter
        self._origin = np.array([float(origin_xy[0]), float(origin_xy[1])],
                                dtype=float)
        angles = self.dirs if dirs is None else np.asarray(dirs, dtype=float).reshape(-1)
        ll = self.lookahead_m if L is None else float(L)
        hits: List[RayHit] = []
        for th in angles:
            d = np.array([math.cos(float(th)), math.sin(float(th))], dtype=float)
            d_occ, d_unk = self._cast_one(adapter, self._origin, d, ll)
            cost = cost_of(float(th), d_occ, d_unk, ll,
                           self.w_ang, self.w_obs, self.w_unk)
            hits.append(RayHit(theta=float(th), d_occ=d_occ, d_unk=d_unk,
                               cost=cost, hit_occ=math.isfinite(d_occ)))
        self._last_hits = hits
        return hits

    def _cast_one(self, adapter: RayGridAdapter, origin: np.ndarray,
                  d: np.ndarray, L: float) -> Tuple[float, float]:
        """沿单位方向 d 投射，返回 (d_occ, d_unk)，未命中为 +inf。

        ⚠ `gimbal_common.math_utils.ray_cells` 的 `origin_xy` 是**栅格局部米制坐标**
        （世界坐标 − 栅格原点），所以这里必须先减去 `adapter.origin_xy`，
        否则栅格原点非 (0,0)（默认场地是 (-6,-6)）时会整体偏移 6m。
        """
        if L <= 0.0:
            return _INF, _INF
        R = rotz2(-adapter.origin_yaw)
        local_origin = R @ (origin - adapter.origin_xy)
        cells = ray_cells(local_origin, R @ d, L, adapter.resolution)
        d_occ = _INF
        d_unk = _INF
        seen = set()
        for (i, j) in cells:
            if (i, j) in seen:
                continue
            seen.add((i, j))
            dist = float(np.dot(adapter.cell_to_world((i, j)) - origin, d))
            if dist > L:
                continue
            dist = max(0.0, min(dist, L))
            if not adapter.in_bounds((i, j)):
                # 越界按未知处理（保守）
                if dist < d_unk:
                    d_unk = dist
                continue
            v = adapter.at((i, j))
            if v == 100:
                if dist < d_occ:
                    d_occ = dist
            elif v < 0:
                if dist < d_unk:
                    d_unk = dist
        return d_occ, d_unk

    # -- 选择 -----------------------------------------------------------------
    def evaluate(self, origin_xy: Sequence[float], u: Sequence[float],
                 L: Optional[float] = None) -> Tuple[float, np.ndarray, bool, List[RayHit]]:
        """完整评估：以 `u` 为基准方向，对 13 个偏转方向投射并选最优。

        Args:
            origin_xy: 射线起点（底盘位置，m）。
            u: 基准单位方向（目标方向）。
            L: 前瞻距离；None 用 `self.lookahead_m`。

        Returns:
            tuple：(theta_star, u_safe, blocked, hits)
              - theta_star: 最优偏转角（rad）
              - u_safe: `Rot(θ*)·u`（2,）ndarray
              - blocked: 全部方向都在 L 内命中占用格
              - hits: 各方向的 RayHit 列表
        """
        uv = np.asarray(u, dtype=float).reshape(2)
        n = float(np.linalg.norm(uv))
        origin = np.array([float(origin_xy[0]), float(origin_xy[1])], dtype=float)
        if self._adapter is None:
            return 0.0, (uv / n if n > 1e-9 else np.zeros(2)), False, []
        if n < 1e-9:
            # 方向退化：直接按 +x 投射，输出零速方向
            hits = self.cast(self._adapter, origin, self.dirs, L)
            blocked = all(h.hit_occ for h in hits)
            return 0.0, np.zeros(2, dtype=float), blocked, hits
        uu = uv / n
        ll = self.lookahead_m if L is None else float(L)

        hits: List[RayHit] = []
        for th in self.dirs:
            d = rotz2(float(th)) @ uu
            d_occ, d_unk = self._cast_one(self._adapter, origin, d, ll)
            cost = cost_of(float(th), d_occ, d_unk, ll,
                           self.w_ang, self.w_obs, self.w_unk)
            hits.append(RayHit(theta=float(th), d_occ=d_occ, d_unk=d_unk,
                               cost=cost, hit_occ=math.isfinite(d_occ)))
        self._last_hits = hits
        return self._pick(hits, uu)

    def choose(self, u: Sequence[float]) -> Tuple[float, np.ndarray, bool]:
        """用最近一次 `cast()` 的结果选择方向（类图接口）。

        Args:
            u: 基准单位方向。

        Returns:
            tuple：(theta_star, u_safe, blocked)
        """
        uv = np.asarray(u, dtype=float).reshape(2)
        n = float(np.linalg.norm(uv))
        uu = uv / n if n > 1e-9 else np.zeros(2, dtype=float)
        if not self._last_hits:
            return 0.0, uu, False
        theta, u_safe, blocked, _hits = self._pick(self._last_hits, uu)
        return theta, u_safe, blocked

    def _pick(self, hits: List[RayHit],
              uu: np.ndarray) -> Tuple[float, np.ndarray, bool, List[RayHit]]:
        """在 hits 中取代价最小者（平局时取 |θ| 最小）。"""
        if not hits:
            return 0.0, uu, False, hits
        best = hits[0]
        for h in hits[1:]:
            if h.cost < best.cost - 1e-12:
                best = h
            elif abs(h.cost - best.cost) <= 1e-12 and abs(h.theta) < abs(best.theta):
                best = h
        blocked = all(h.hit_occ for h in hits)
        u_safe = rotz2(best.theta) @ uu
        return best.theta, u_safe, blocked, hits
