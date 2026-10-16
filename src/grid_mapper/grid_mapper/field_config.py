"""field.yaml 解析：场地边界多边形 + rect / circle / segment 障碍。

纯 Python（不依赖 rclpy），可直接单测。对应 DESIGN §10 的 `field.yaml` 结构约定：

```yaml
grid:
  resolution: 0.05
  width_m: 12.0
  height_m: 12.0
  origin_xyz: [-6.0, -6.0, 0.0]
  inflation_radius_m: 0.10
boundary:                       # 边界多边形（map 系，逆时针）
  - [-6.0, -6.0]
  - [ 6.0, -6.0]
  - [ 6.0,  6.0]
  - [-6.0,  6.0]
obstacles:
  - type: rect
    center_xy: [2.0, 1.0]
    size_xy: [0.6, 0.6]
    yaw: 0.0
  - type: circle
    center_xy: [-1.5, 2.0]
    radius: 0.4
  - type: segment
    p0_xy: [-3.0, -2.0]
    p1_xy: [ 1.0, -2.0]
    width: 0.1
```

坐标全部在 **map 系**（给定点居中，12×12m 场地 → origin = [-6, -6, 0]）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from dataclasses import field as dc_field
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np

try:  # 优先用 T01 的 gimbal_common
    from gimbal_common.config_loader import load_yaml
except Exception:  # pragma: no cover - Windows 无 ROS2 时的兜底
    from ._compat import load_yaml

__all__ = [
    'DEFAULT_FIELD',
    'Obstacle',
    'FieldConfig',
    'point_in_polygon',
    'rect_contains',
    'circle_contains',
    'segment_contains',
    'masks_from_field',
]

# 缺省场地（DESIGN §10 原样），field.yaml 缺失/字段缺失时的兜底值
DEFAULT_FIELD: Dict[str, Any] = {
    'grid': {
        'resolution': 0.05,
        'width_m': 12.0,
        'height_m': 12.0,
        'origin_xyz': [-6.0, -6.0, 0.0],
        'inflation_radius_m': 0.10,
    },
    'boundary': [[-6.0, -6.0], [6.0, -6.0], [6.0, 6.0], [-6.0, 6.0]],
    'obstacles': [],
}


# ---------------------------------------------------------------------------
# 几何判定（纯函数，numpy 矢量化版本在 masks_from_field 中）
# ---------------------------------------------------------------------------
def point_in_polygon(x: float, y: float, poly: np.ndarray) -> bool:
    """射线法判定点是否在多边形内（含边界返回 True）。

    Args:
        x: 点的 x 坐标。
        y: 点的 y 坐标。
        poly: 形状 (N, 2) 的多边形顶点数组（顺序任意，闭合与否均可）。

    Returns:
        bool：点在多边形内（含边界）为 True。
    """
    if poly is None or len(poly) < 3:
        return True
    px = float(x)
    py = float(y)
    n = int(poly.shape[0])
    inside = False
    on_edge = False
    for k in range(n):
        x0 = float(poly[k, 0])
        y0 = float(poly[k, 1])
        x1 = float(poly[(k + 1) % n, 0])
        y1 = float(poly[(k + 1) % n, 1])
        # 边界（含顶点）判定
        cross = (x1 - x0) * (py - y0) - (y1 - y0) * (px - x0)
        if abs(cross) <= 1e-12:
            if (min(x0, x1) - 1e-12 <= px <= max(x0, x1) + 1e-12 and
                    min(y0, y1) - 1e-12 <= py <= max(y0, y1) + 1e-12):
                on_edge = True
                break
        if (y0 > py) != (y1 > py):
            xint = (x1 - x0) * (py - y0) / ((y1 - y0) + 1e-30) + x0
            if px < xint:
                inside = not inside
    return bool(on_edge or inside)


def rect_contains(xy: np.ndarray, center_xy: Sequence[float],
                  size_xy: Sequence[float], yaw: float = 0.0) -> bool:
    """判定点是否在（可旋转）矩形内，含边界。"""
    cx = float(center_xy[0])
    cy = float(center_xy[1])
    sx = float(size_xy[0])
    sy = float(size_xy[1])
    dx = float(xy[0]) - cx
    dy = float(xy[1]) - cy
    c = math.cos(-float(yaw))
    s = math.sin(-float(yaw))
    lx = c * dx - s * dy
    ly = s * dx + c * dy
    return bool(abs(lx) <= 0.5 * abs(sx) + 1e-12 and abs(ly) <= 0.5 * abs(sy) + 1e-12)


def circle_contains(xy: np.ndarray, center_xy: Sequence[float],
                    radius: float) -> bool:
    """判定点是否在圆内（含边界）。"""
    dx = float(xy[0]) - float(center_xy[0])
    dy = float(xy[1]) - float(center_xy[1])
    return bool(dx * dx + dy * dy <= float(radius) * float(radius) + 1e-12)


def segment_contains(xy: np.ndarray, p0_xy: Sequence[float],
                     p1_xy: Sequence[float], width: float) -> bool:
    """判定点到线段的距离是否 ≤ width/2（即被线段型"墙"覆盖）。"""
    ax = float(p0_xy[0])
    ay = float(p0_xy[1])
    bx = float(p1_xy[0])
    by = float(p1_xy[1])
    px = float(xy[0])
    py = float(xy[1])
    vx = bx - ax
    vy = by - ay
    den = vx * vx + vy * vy
    if den < 1e-18:                       # 退化成点
        d2 = (px - ax) ** 2 + (py - ay) ** 2
    else:
        t = ((px - ax) * vx + (py - ay) * vy) / den
        t = max(0.0, min(1.0, t))
        qx = ax + t * vx
        qy = ay + t * vy
        d2 = (px - qx) ** 2 + (py - qy) ** 2
    return bool(d2 <= (0.5 * float(width)) ** 2 + 1e-12)


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------
@dataclass
class Obstacle:
    """单个障碍物描述。

    Attributes:
        kind: 'rect' | 'circle' | 'segment'。
        center_xy: 矩形/圆的中心（segment 未使用，置为 (0,0)）。
        size_xy: 矩形尺寸 (sx, sy)，单位 m。
        yaw: 矩形绕中心逆时针旋转角，单位 rad。
        radius: 圆半径，单位 m。
        p0_xy / p1_xy: 线段两端点，单位 m。
        width: 线段宽度（墙厚），单位 m。
    """

    kind: str = 'rect'
    center_xy: Tuple[float, float] = (0.0, 0.0)
    size_xy: Tuple[float, float] = (0.0, 0.0)
    yaw: float = 0.0
    radius: float = 0.0
    p0_xy: Tuple[float, float] = (0.0, 0.0)
    p1_xy: Tuple[float, float] = (0.0, 0.0)
    width: float = 0.0

    # -- 矢量化判定：输入 (H, W, 2) 的世界坐标网格，返回 bool mask -------------
    def mask(self, xy: np.ndarray) -> np.ndarray:
        """返回本障碍覆盖的 bool mask（shape 与 xy 的前两维一致）。"""
        k = str(self.kind).lower()
        if k == 'rect':
            return _rect_mask(xy, self.center_xy, self.size_xy, self.yaw)
        if k == 'circle':
            return _circle_mask(xy, self.center_xy, self.radius)
        if k == 'segment':
            return _segment_mask(xy, self.p0_xy, self.p1_xy, self.width)
        return np.zeros(xy.shape[:2], dtype=bool)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'Obstacle':
        """从 field.yaml 的一个 obstacle 条目构造；未知类型返回 'none'（不占格）。"""
        kind = str(d.get('type', 'rect')).lower()
        obs = Obstacle(kind=kind)
        if kind == 'rect':
            obs.center_xy = _as_vec2(d.get('center_xy', [0.0, 0.0]))
            obs.size_xy = _as_vec2(d.get('size_xy', [0.0, 0.0]))
            obs.yaw = float(d.get('yaw', 0.0))
        elif kind == 'circle':
            obs.center_xy = _as_vec2(d.get('center_xy', [0.0, 0.0]))
            obs.radius = float(d.get('radius', 0.0))
        elif kind == 'segment':
            obs.p0_xy = _as_vec2(d.get('p0_xy', [0.0, 0.0]))
            obs.p1_xy = _as_vec2(d.get('p1_xy', [0.0, 0.0]))
            obs.width = float(d.get('width', 0.0))
        else:
            obs.kind = 'none'
        return obs


@dataclass
class FieldConfig:
    """场地配置（map 系）。

    Attributes:
        resolution: 栅格分辨率，m/cell（默认 0.05）。
        width_m / height_m: 场地尺寸，m（默认 12×12）。
        origin_xyz: map.yaml 的 origin，默认 [-6, -6, 0]（给定点居中）。
        inflation_radius_m: 膨胀半径，m（默认 0.10）。
        boundary: (N, 2) 边界多边形顶点。
        obstacles: 障碍列表。
    """

    resolution: float = 0.05
    width_m: float = 12.0
    height_m: float = 12.0
    origin_xyz: Tuple[float, float, float] = (-6.0, -6.0, 0.0)
    inflation_radius_m: float = 0.10
    boundary: np.ndarray = dc_field(
        default_factory=lambda: np.array(
            [[-6.0, -6.0], [6.0, -6.0], [6.0, 6.0], [-6.0, 6.0]], dtype=float))
    obstacles: List[Obstacle] = dc_field(default_factory=list)

    # -- 派生量 ---------------------------------------------------------------
    @property
    def origin_xy(self) -> np.ndarray:
        """栅格原点（左下角）的 (x, y)。"""
        return np.array([float(self.origin_xyz[0]), float(self.origin_xyz[1])],
                        dtype=float)

    @property
    def width_cells(self) -> int:
        """栅格宽度（cell 数），至少 1。"""
        return max(1, int(round(self.width_m / max(self.resolution, 1e-9))))

    @property
    def height_cells(self) -> int:
        """栅格高度（cell 数），至少 1。"""
        return max(1, int(round(self.height_m / max(self.resolution, 1e-9))))

    @property
    def shape(self) -> Tuple[int, int]:
        """栅格数组形状 (W, H)。"""
        return (self.width_cells, self.height_cells)

    # -- 构造 -----------------------------------------------------------------
    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'FieldConfig':
        """从 dict 构造。缺失字段用 DEFAULT_FIELD 兜底（不报 FATAL）。"""
        d = d if isinstance(d, dict) else {}
        g = d.get('grid', {}) if isinstance(d.get('grid', {}), dict) else {}
        dg = DEFAULT_FIELD['grid']

        res = float(g.get('resolution', dg['resolution']))
        if res <= 0.0:
            res = float(dg['resolution'])
        w_m = float(g.get('width_m', dg['width_m']))
        h_m = float(g.get('height_m', dg['height_m']))
        org = g.get('origin_xyz', dg['origin_xyz'])
        org = _as_vec3(org, (float(dg['origin_xyz'][0]),
                             float(dg['origin_xyz'][1]),
                             float(dg['origin_xyz'][2])))
        infl = float(g.get('inflation_radius_m', dg['inflation_radius_m']))

        bnd_raw = d.get('boundary', DEFAULT_FIELD['boundary'])
        boundary = _as_polygon(bnd_raw)

        obstacles: List[Obstacle] = []
        obs_raw = d.get('obstacles', []) or []
        if isinstance(obs_raw, list):
            for item in obs_raw:
                if isinstance(item, dict):
                    obstacles.append(Obstacle.from_dict(item))

        return FieldConfig(
            resolution=res,
            width_m=w_m,
            height_m=h_m,
            origin_xyz=org,
            inflation_radius_m=max(0.0, infl),
            boundary=boundary,
            obstacles=obstacles,
        )

    @staticmethod
    def from_yaml(path: str) -> 'FieldConfig':
        """从 field.yaml 构造；文件缺失或解析失败时返回默认场地（调用方决定是否 FATAL）。

        ⚠ `gimbal_common.config_loader.load_yaml(path, required=True)` 在文件缺失时
        **抛 ConfigError**（FATAL）。建图节点允许"没配就用默认场地"，所以这里
        显式传 `required=False`，并额外 try/except 兜住抛异常的实现。
        """
        data = None
        try:
            data = load_yaml(path, required=False)
        except TypeError:  # pragma: no cover - 旧签名 load_yaml(path)
            try:
                data = load_yaml(path)
            except Exception:
                data = None
        except Exception:  # 文件缺失 / 内容非映射 / YAML 语法错
            data = None
        if not isinstance(data, dict):
            return FieldConfig.from_dict(DEFAULT_FIELD)
        return FieldConfig.from_dict(data)

    @staticmethod
    def default() -> 'FieldConfig':
        """返回 DESIGN §10 的默认场地（12×12m，给定点居中，无障碍）。"""
        return FieldConfig.from_dict(DEFAULT_FIELD)

    # -- 判定 -----------------------------------------------------------------
    def inside_boundary(self, x: float, y: float) -> bool:
        """点是否在边界多边形内（标量版本，便于单测与调试）。"""
        return point_in_polygon(x, y, self.boundary)

    def obstacle_mask(self, xy: np.ndarray) -> np.ndarray:
        """返回所有障碍的并集 bool mask（shape = xy.shape[:2]）。"""
        out = np.zeros(xy.shape[:2], dtype=bool)
        for obs in self.obstacles:
            out |= obs.mask(xy)
        return out


# ---------------------------------------------------------------------------
# 矢量化 mask 辅助（内部）
# ---------------------------------------------------------------------------
def _rect_mask(xy: np.ndarray, center_xy: Sequence[float],
               size_xy: Sequence[float], yaw: float) -> np.ndarray:
    cx = float(center_xy[0])
    cy = float(center_xy[1])
    sx = abs(float(size_xy[0]))
    sy = abs(float(size_xy[1]))
    dx = xy[..., 0] - cx
    dy = xy[..., 1] - cy
    c = math.cos(-float(yaw))
    s = math.sin(-float(yaw))
    lx = c * dx - s * dy
    ly = s * dx + c * dy
    return (np.abs(lx) <= 0.5 * sx) & (np.abs(ly) <= 0.5 * sy)


def _circle_mask(xy: np.ndarray, center_xy: Sequence[float],
                 radius: float) -> np.ndarray:
    dx = xy[..., 0] - float(center_xy[0])
    dy = xy[..., 1] - float(center_xy[1])
    r = abs(float(radius))
    return (dx * dx + dy * dy) <= (r * r)


def _segment_mask(xy: np.ndarray, p0_xy: Sequence[float],
                  p1_xy: Sequence[float], width: float) -> np.ndarray:
    ax = float(p0_xy[0])
    ay = float(p0_xy[1])
    bx = float(p1_xy[0])
    by = float(p1_xy[1])
    vx = bx - ax
    vy = by - ay
    den = vx * vx + vy * vy
    px = xy[..., 0] - ax
    py = xy[..., 1] - ay
    if den < 1e-18:
        d2 = px * px + py * py
    else:
        t = (px * vx + py * vy) / den
        t = np.clip(t, 0.0, 1.0)
        qx = px - t * vx
        qy = py - t * vy
        d2 = qx * qx + qy * qy
    half = 0.5 * abs(float(width))
    return d2 <= (half * half)


def _polygon_mask(xy: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """矢量化"点在多边形内"（叉积同号法，支持凸/凹多边形；退化时全 True）。"""
    if poly is None or poly.shape[0] < 3:
        return np.ones(xy.shape[:2], dtype=bool)
    n = int(poly.shape[0])
    # 偶奇规则（射线法）的矢量化版本：对凹多边形同样成立
    inside = np.zeros(xy.shape[:2], dtype=bool)
    for k in range(n):
        x0 = float(poly[k, 0])
        y0 = float(poly[k, 1])
        x1 = float(poly[(k + 1) % n, 0])
        y1 = float(poly[(k + 1) % n, 1])
        if abs(y1 - y0) < 1e-18:
            continue                       # 水平边不产生交点
        cond = ((y0 > xy[..., 1]) != (y1 > xy[..., 1]))
        xint = (x1 - x0) * (xy[..., 1] - y0) / (y1 - y0) + x0
        inside = np.logical_xor(inside, cond & (xy[..., 0] < xint))
    # 边界上的点：叉积接近 0 且在边的包围盒内 → 计入内部
    for k in range(n):
        x0 = float(poly[k, 0])
        y0 = float(poly[k, 1])
        x1 = float(poly[(k + 1) % n, 0])
        y1 = float(poly[(k + 1) % n, 1])
        cross = (x1 - x0) * (xy[..., 1] - y0) - (y1 - y0) * (xy[..., 0] - x0)
        near = np.abs(cross) <= 1e-9
        inx = (xy[..., 0] >= min(x0, x1) - 1e-9) & (xy[..., 0] <= max(x0, x1) + 1e-9)
        iny = (xy[..., 1] >= min(y0, y1) - 1e-9) & (xy[..., 1] <= max(y0, y1) + 1e-9)
        inside |= (near & inx & iny)
    return inside


def masks_from_field(field_cfg: FieldConfig) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """生成 (inside_mask, obstacle_mask, xy_grid)。

    Args:
        field_cfg: 场地配置。

    Returns:
        inside_mask: bool 数组 (W, H)，cell 中心在边界多边形内为 True。
        obstacle_mask: bool 数组 (W, H)，cell 中心落在障碍内为 True。
        xy_grid: float 数组 (W, H, 2)，每个 cell 中心的世界坐标。
    """
    res = float(field_cfg.resolution)
    origin = field_cfg.origin_xy
    w = field_cfg.width_cells
    h = field_cfg.height_cells
    ii = np.arange(w, dtype=float)          # i -> x
    jj = np.arange(h, dtype=float)          # j -> y
    xs = origin[0] + (ii + 0.5) * res
    ys = origin[1] + (jj + 0.5) * res
    xg, yg = np.meshgrid(xs, ys, indexing='ij')      # (W, H)
    xy = np.stack([xg, yg], axis=-1)                 # (W, H, 2)
    inside = _polygon_mask(xy, field_cfg.boundary)
    obs = field_cfg.obstacle_mask(xy) & inside
    return inside, obs, xy


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _as_vec2(v: Any, default: Tuple[float, float] = (0.0, 0.0)) -> Tuple[float, float]:
    """把任意序列转成 (float, float)。"""
    try:
        seq = list(v)
    except TypeError:
        return default
    if len(seq) < 2:
        return default
    return (float(seq[0]), float(seq[1]))


def _as_vec3(v: Any,
             default: Tuple[float, float, float] = (0.0, 0.0, 0.0)
             ) -> Tuple[float, float, float]:
    """把任意序列转成 (float, float, float)。"""
    try:
        seq = list(v)
    except TypeError:
        return default
    if len(seq) < 3:
        return (float(seq[0]), float(seq[1]), float(default[2]))
    return (float(seq[0]), float(seq[1]), float(seq[2]))


def _as_polygon(v: Any) -> np.ndarray:
    """把 [[x,y], ...] 转成 (N,2) float 数组；非法时退回默认矩形。"""
    if not isinstance(v, (list, tuple)) or len(v) < 3:
        return np.array(DEFAULT_FIELD['boundary'], dtype=float)
    pts: List[List[float]] = []
    for p in v:
        try:
            seq = list(p)
        except TypeError:
            continue
        if len(seq) >= 2:
            pts.append([float(seq[0]), float(seq[1])])
    if len(pts) < 3:
        return np.array(DEFAULT_FIELD['boundary'], dtype=float)
    return np.array(pts, dtype=float)


def occupancy_grid_to_arrays(width: int, height: int, resolution: float,
                             origin_xy: Sequence[float],
                             data: Sequence[int]) -> Tuple[np.ndarray, np.ndarray]:
    """把外部 `nav_msgs/OccupancyGrid`（map_source=topic）转成内部栅格。

    OccupancyGrid 的 `data` 是 **行主序、按行从下往上**？—— 实际上 ROS 约定
    `data[i]` 的 index = `x + y * width`，其中 y=0 对应 origin 那一行。
    本函数按该约定还原成 (W, H) 的本包内部排布。

    Args:
        width: 栅格宽度（cell）。
        height: 栅格高度（cell）。
        resolution: 分辨率 m/cell。
        origin_xy: 原点 (x, y)。
        data: 长度 width*height 的 int8 序列（-1/0/100 或 0..100）。

    Returns:
        (grid, grid_copy)：`grid` 为 (W, H) int8 三态数组；第二个返回值恒等于 `grid`
        （历史兼容，勿再当作"含膨胀的代价图"；要膨胀请走
        `GridMap.from_occupancy_grid(..., inflation_radius_m=...)`）。
    """
    if width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('OccupancyGrid dimensions and resolution must be positive')
    values = np.asarray(list(data), dtype=float)
    if values.size != width * height:
        raise ValueError('OccupancyGrid data length must equal width*height')
    if not np.all(np.isfinite(values)) or np.any(values < -1) or np.any(values > 100) or np.any(values != np.floor(values)):
        raise ValueError('OccupancyGrid values must be integers in [-1,100]')
    arr = values.astype(np.int8).reshape((height, width))
    grid = np.transpose(arr)
    # Match saved Nav2 trinary thresholds; ambiguous probability remains unknown.
    grid = np.where(grid < 0, -1, np.where(grid >= 65, 100,
                    np.where(grid <= 19, 0, -1))).astype(np.int8)
    return grid, grid.copy()
