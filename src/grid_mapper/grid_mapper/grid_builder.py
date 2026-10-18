"""栅格化：三态（-1 未知 / 0 空闲 / 100 占用）+ 膨胀代价图 + 世界↔cell 互转。

纯 Python（不依赖 rclpy），可直接单测。规格见 DESIGN §6.11：

- 分辨率 0.05 m/cell，12×12m → 240×240，`map.yaml` origin `[-6,-6,0]`（给定点居中）
- 三态：未知 `-1`(PGM 205) / 空闲 `0`(PGM 254) / 占用 `100`(PGM 0)
- 膨胀半径 0.10m（2 cells），**只进 `ray_grid`，不写 pgm**
- cell 中心世界坐标：`c(i,j) = origin_xy + ([i+0.5, j+0.5] * res)`
- 栅格化规则：边界多边形外 → -1；障碍内部 → 100；其余 → 0
- ray_grid：`grid==100` 或 到障碍距离 ≤ `inflation_radius` → 100；`grid==-1` → -1；否则 0

⚠ 数组排布：`grid[i, j]`，shape = **(W, H)**（第一维是 x 方向 i，第二维是 y 方向 j），
与 DESIGN §6.11 的 PGM 片段 `row = grid[:, (H-1-r)]` 一致。
"""

from __future__ import annotations

import math
from typing import Any
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np

from gimbal_common.math_utils import clamp, ray_cells

from .field_config import FieldConfig, masks_from_field, occupancy_grid_to_arrays

__all__ = [
    'UNKNOWN',
    'FREE',
    'OCCUPIED',
    'PGM_UNKNOWN',
    'PGM_FREE',
    'PGM_OCCUPIED',
    'GridMap',
    'inflate',
    'ray_cells',
]

# 三态取值（OccupancyGrid 语义）
UNKNOWN: int = -1
FREE: int = 0
OCCUPIED: int = 100

# 对应的 PGM 灰度（DESIGN §6.11）
PGM_UNKNOWN: int = 205
PGM_FREE: int = 254
PGM_OCCUPIED: int = 0


class GridMap:
    """栅格地图容器（map 系）。

    Attributes:
        resolution: 分辨率，m/cell。
        origin_xy: 栅格左下角原点 (x, y)，m。
        grid: int8 数组，shape (W, H)，取值 {-1, 0, 100}（干净三态）。
        ray_grid: int8 数组，shape (W, H)，含膨胀，供避障射线投射使用。
        inflation_radius_m: 生成 ray_grid 时使用的膨胀半径，m。
    """

    def __init__(self,
                 resolution: float = 0.05,
                 origin_xy: Sequence[float] = (-6.0, -6.0),
                 grid: Optional[np.ndarray] = None,
                 ray_grid: Optional[np.ndarray] = None,
                 inflation_radius_m: float = 0.10,
                 origin_yaw: float = 0.0) -> None:
        if not math.isfinite(float(resolution)) or float(resolution) <= 0:
            raise ValueError('map resolution must be positive and finite')
        self.resolution = float(resolution)
        self.origin_yaw = float(origin_yaw)
        if not math.isfinite(self.origin_yaw) or not np.all(np.isfinite(origin_xy)):
            raise ValueError('map origin must be finite')
        self.origin_xy = np.array([float(origin_xy[0]), float(origin_xy[1])],
                                  dtype=float)
        if grid is None:
            grid = np.full((1, 1), UNKNOWN, dtype=np.int8)
        self.grid = np.asarray(grid, dtype=np.int8)
        if self.grid.ndim != 2:
            raise ValueError('grid must be 2-D, got shape %r' % (self.grid.shape,))
        if ray_grid is None:
            ray_grid = self.grid.copy()
        self.ray_grid = np.asarray(ray_grid, dtype=np.int8)
        if self.ray_grid.shape != self.grid.shape:
            raise ValueError('ray_grid shape %r != grid shape %r'
                             % (self.ray_grid.shape, self.grid.shape))
        self.inflation_radius_m = float(inflation_radius_m)

    # -- 基本尺寸 -------------------------------------------------------------
    @property
    def width(self) -> int:
        """栅格宽度（cell 数）。"""
        return int(self.grid.shape[0])

    @property
    def height(self) -> int:
        """栅格高度（cell 数）。"""
        return int(self.grid.shape[1])

    @property
    def shape(self) -> Tuple[int, int]:
        """栅格形状 (W, H)。"""
        return (self.width, self.height)

    @property
    def width_m(self) -> float:
        """场地宽度，m。"""
        return self.width * self.resolution

    @property
    def height_m(self) -> float:
        """场地高度，m。"""
        return self.height * self.resolution

    # -- 坐标换算 -------------------------------------------------------------
    def world_to_cell(self, xy: Sequence[float]) -> Tuple[int, int]:
        """世界(map)坐标 -> cell 索引 `(i, j)`（向下取整）。

        Args:
            xy: (x, y)，单位 m。

        Returns:
            tuple[int, int]：(i, j)。不做边界裁剪，可能越界。
        """
        x = float(xy[0])
        y = float(xy[1])
        dx, dy = x-self.origin_xy[0], y-self.origin_xy[1]
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        i = int(math.floor((c*dx+s*dy) / self.resolution))
        j = int(math.floor((-s*dx+c*dy) / self.resolution))
        return (i, j)

    def world_to_cell_clamped(self, xy: Sequence[float]) -> Tuple[int, int]:
        """同 `world_to_cell`，但把结果裁剪到合法范围。"""
        i, j = self.world_to_cell(xy)
        i = int(clamp(i, 0, self.width - 1))
        j = int(clamp(j, 0, self.height - 1))
        return (i, j)

    def cell_to_world(self, ij: Sequence[int]) -> np.ndarray:
        """cell 索引 `(i, j)` -> 该 cell **中心**的世界坐标 ndarray(2,)。"""
        i = int(ij[0])
        j = int(ij[1])
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        local = np.array([i+0.5, j+0.5]) * self.resolution
        return self.origin_xy + np.array([[c, -s], [s, c]]) @ local

    def in_bounds(self, ij: Sequence[int]) -> bool:
        """cell 索引是否落在栅格范围内。"""
        i = int(ij[0])
        j = int(ij[1])
        return bool(0 <= i < self.width and 0 <= j < self.height)

    def world_in_bounds(self, xy: Sequence[float]) -> bool:
        """世界坐标是否落在栅格范围内。"""
        return self.in_bounds(self.world_to_cell(xy))

    # -- 取值 -----------------------------------------------------------------
    def at(self, ij: Sequence[int], use_ray: bool = False) -> int:
        """取某个 cell 的值；越界返回 UNKNOWN(-1)。"""
        if not self.in_bounds(ij):
            return UNKNOWN
        src = self.ray_grid if use_ray else self.grid
        return int(src[int(ij[0]), int(ij[1])])

    def at_world(self, xy: Sequence[float], use_ray: bool = False) -> int:
        """取世界坐标所在 cell 的值；越界返回 UNKNOWN(-1)。"""
        return self.at(self.world_to_cell(xy), use_ray=use_ray)

    # -- 构造 -----------------------------------------------------------------
    @staticmethod
    def build(field_cfg: FieldConfig) -> 'GridMap':
        """按 DESIGN §6.11 从场地配置栅格化，同时生成膨胀后的 `ray_grid`。

        Args:
            field_cfg: 场地配置（边界多边形 + 障碍列表 + 栅格参数）。

        Returns:
            GridMap：含干净三态 `grid` 与膨胀代价 `ray_grid`。
        """
        if abs(float(field_cfg.origin_xyz[2])) > 1e-10:
            raise ValueError('config maps currently require zero grid origin yaw; use external topic for rotated grids')
        inside, obs, _xy = masks_from_field(field_cfg)
        grid = np.where(inside, np.where(obs, OCCUPIED, FREE), UNKNOWN).astype(np.int8)
        ray_grid = inflate(grid, field_cfg.inflation_radius_m, field_cfg.resolution)
        return GridMap(resolution=field_cfg.resolution,
                       origin_xy=field_cfg.origin_xy,
                       grid=grid,
                       ray_grid=ray_grid,
                       inflation_radius_m=field_cfg.inflation_radius_m)

    @staticmethod
    def from_arrays(grid: np.ndarray,
                    resolution: float,
                    origin_xy: Sequence[float],
                    inflation_radius_m: float = 0.0,
                    origin_yaw: float = 0.0) -> 'GridMap':
        """从已有栅格数组构造（map_source=topic 或单测用）。

        `inflation_radius_m > 0` 时会重新生成 `ray_grid`；否则 `ray_grid = grid`。
        """
        g = np.asarray(grid, dtype=np.int8)
        if inflation_radius_m > 0.0:
            ray_grid = inflate(g, inflation_radius_m, resolution)
        else:
            ray_grid = g.copy()
        return GridMap(resolution=resolution, origin_xy=origin_xy,
                       grid=g, ray_grid=ray_grid,
                       inflation_radius_m=float(inflation_radius_m), origin_yaw=origin_yaw)

    @staticmethod
    def from_occupancy_grid(width: int, height: int, resolution: float,
                            origin_xy: Sequence[float],
                            data: Sequence[int],
                            inflation_radius_m: float = 0.0,
                    origin_yaw: float = 0.0) -> 'GridMap':
        """从外部 `nav_msgs/OccupancyGrid` 的数据构造（`grid.map_source=topic`）。

        ⚠ **原样保留外部地图的 `resolution` / `origin`**（SLAM 给出什么就是什么），
        **不**重采样到 `grid.width_m/height_m/origin_xyz` 那个 12×12 m 窗口——
        两者是两回事，强行套参数里的 origin 会让整张图平移。

        `inflation_radius_m > 0` 时**只对 `ray_grid` 膨胀**：`grid` 保持原始三态
        （方向项要用未膨胀的图算可行射线），`ray_grid` 才是避障用的含膨胀代价图。

        Args:
            width: 栅格宽度（cell）。
            height: 栅格高度（cell）。
            resolution: 分辨率 m/cell。
            origin_xy: 外部地图原点 (x, y)，m。
            data: 长度 width*height 的 int8 序列。
            inflation_radius_m: 膨胀半径 m；0 = 不膨胀。

        Returns:
            GridMap：`grid` 为原始三态，`ray_grid` 为（可选的）膨胀代价图。
        """
        g, _r = occupancy_grid_to_arrays(width, height, resolution, origin_xy, data)
        if inflation_radius_m > 0.0:
            ray = inflate(g, float(inflation_radius_m), float(resolution))
        else:
            ray = g.copy()
        return GridMap(resolution=resolution, origin_xy=origin_xy,
                       grid=g, ray_grid=ray,
                       inflation_radius_m=float(inflation_radius_m), origin_yaw=origin_yaw)

    # -- 导出 -----------------------------------------------------------------
    def to_occupancy_data(self, use_ray: bool = False) -> List[int]:
        """导出为 `nav_msgs/OccupancyGrid.data`（index = x + y*width，行主序）。"""
        src = self.ray_grid if use_ray else self.grid
        # grid 是 [i, j] = [x, y]；转置成 [y, x] 后展平即 ROS 约定顺序
        return np.transpose(src).reshape(-1).astype(np.int8).tolist()

    def stats(self) -> dict:
        """统计三态 cell 数量（用于日志与自检）。"""
        g = self.grid
        return {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin_xy': [float(self.origin_xy[0]), float(self.origin_xy[1])],
            'origin_yaw': self.origin_yaw,
            'n_occupied': int(np.count_nonzero(g == OCCUPIED)),
            'n_free': int(np.count_nonzero(g == FREE)),
            'n_unknown': int(np.count_nonzero(g == UNKNOWN)),
            'n_ray_occupied': int(np.count_nonzero(self.ray_grid == OCCUPIED)),
        }

    def __repr__(self) -> str:  # pragma: no cover - 仅调试
        return ('GridMap(W=%d, H=%d, res=%.3f, origin=(%.2f, %.2f), '
                'occ=%d, free=%d, unk=%d)'
                % (self.width, self.height, self.resolution,
                   self.origin_xy[0], self.origin_xy[1],
                   self.stats()['n_occupied'], self.stats()['n_free'],
                   self.stats()['n_unknown']))


# ---------------------------------------------------------------------------
# 膨胀
# ---------------------------------------------------------------------------
def inflate(grid: np.ndarray, radius_m: float, resolution: float) -> np.ndarray:
    """生成膨胀代价图 `ray_grid`（只用于避障，不写 pgm）。

    规则（DESIGN §6.11）：
      - `grid == 100`            → 100
      - `grid == -1`             → -1（未知区不膨胀，保持未知）
      - 空闲但到障碍 cell **中心距** ≤ radius_m → 100
      - 其余 → 0

    Args:
        grid: (W, H) int8 三态数组。
        radius_m: 膨胀半径，m。
        resolution: 分辨率，m/cell。

    Returns:
        np.ndarray：(W, H) int8 膨胀后的代价图。
    """
    g = np.asarray(grid, dtype=np.int8)
    out = g.copy()
    if radius_m <= 0.0 or resolution <= 0.0:
        return out
    radius_cells = radius_m / resolution
    if radius_cells <= 0.0:
        return out
    occ = (g == OCCUPIED)
    if not np.any(occ):
        return out
    free = (g == FREE)
    grown = _dilate(occ, radius_cells)
    out = np.where(g == UNKNOWN, UNKNOWN,
                   np.where(g == OCCUPIED, OCCUPIED,
                            np.where(free & grown, OCCUPIED, FREE))).astype(np.int8)
    return out


def _dilate(mask: np.ndarray, radius_cells: float) -> np.ndarray:
    """用显式偏移做欧氏圆盘膨胀（中心距 ≤ radius_cells 的偏移全部置 1）。

    对 240×240、半径 2 cells 的规模，偏移数仅 13 个，代价可忽略。
    """
    m = np.asarray(mask, dtype=bool)
    w, h = m.shape[0], m.shape[1]
    out = m.copy()
    r = int(math.ceil(radius_cells - 1e-9))
    r = max(0, min(r, max(w, h)))
    r2 = radius_cells * radius_cells + 1e-9
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di == 0 and dj == 0:
                continue
            if float(di * di + dj * dj) > r2:
                continue
            out |= _shift(m, di, dj)
    return out


def _shift(mask: np.ndarray, di: int, dj: int) -> np.ndarray:
    """把 bool mask 平移 (di, dj)，越界补 False。等价于 out[i,j] = mask[i-di, j-dj]。"""
    m = np.asarray(mask, dtype=bool)
    w, h = m.shape[0], m.shape[1]
    out = np.zeros((w, h), dtype=bool)
    src_i0 = max(0, -di)
    src_i1 = min(w, w - di)
    src_j0 = max(0, -dj)
    src_j1 = min(h, h - dj)
    if src_i1 <= src_i0 or src_j1 <= src_j0:
        return out
    dst_i0 = src_i0 + di
    dst_i1 = src_i1 + di
    dst_j0 = src_j0 + dj
    dst_j1 = src_j1 + dj
    out[dst_i0:dst_i1, dst_j0:dst_j1] = m[src_i0:src_i1, src_j0:src_j1]
    return out
