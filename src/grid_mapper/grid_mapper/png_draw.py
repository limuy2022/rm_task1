"""`map_preview.png` / `map_tracked.png` 绘制（Pillow）。

纯 Python（不依赖 rclpy），依赖 Pillow 输出 PNG。

颜色约定（DESIGN §6.11）：

| 要素 | 颜色 |
|---|---|
| 占用 | 黑 `(0,0,0)` |
| 空闲 | 白 `(255,255,255)` |
| 未知 | 灰 `(205,205,205)` |
| 场地边界 | 蓝 `(0,0,255)` |
| 给定点（map 原点） | 品红十字 `(255,0,255)` |
| 目标估计轨迹 | 蓝 `(0,0,255)` |
| 目标当前位置 | 红点 `(255,0,0)` |
| 目标预测位置 | 红× `(255,0,0)` |
| 底盘轨迹 | 绿 `(0,180,0)` |
| 底盘当前位置 | 绿方块 `(0,180,0)` |

⚠ 图像行序与 PGM 一致：**顶行 = y 最大**（见 `pgm_io.grid_to_pgm_image`）。
"""

from __future__ import annotations

import math
import os
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np

from PIL import Image, ImageDraw

from .grid_builder import GridMap
from .pgm_io import grid_to_pgm_image

__all__ = [
    'world_to_pixel',
    'base_image',
    'draw_preview',
    'draw_tracked',
    'downsample',
]

# 配色（R, G, B）
C_OCC = (0, 0, 0)
C_FREE = (255, 255, 255)
C_UNK = (205, 205, 205)
C_BOUNDARY = (0, 0, 255)
C_GIVEN = (255, 0, 255)
C_TARGET_TRAJ = (0, 0, 255)
C_TARGET_CUR = (255, 0, 0)
C_TARGET_PRED = (255, 0, 0)
C_BASE_TRAJ = (0, 180, 0)
C_BASE_CUR = (0, 180, 0)


# ---------------------------------------------------------------------------
# 坐标换算
# ---------------------------------------------------------------------------
def world_to_pixel(xy: Sequence[float], grid_map: GridMap) -> Tuple[float, float]:
    """世界(map)坐标 -> 图像像素坐标 `(px, py)`（py = 0 是图像顶行 = y 最大）。"""
    dx, dy = float(xy[0])-grid_map.origin_xy[0], float(xy[1])-grid_map.origin_xy[1]
    c, s = math.cos(grid_map.origin_yaw), math.sin(grid_map.origin_yaw)
    fx = (c*dx+s*dy) / grid_map.resolution - 0.5
    fy = (-s*dx+c*dy) / grid_map.resolution - 0.5
    py = (grid_map.height - 1) - fy
    return (fx, py)


# ---------------------------------------------------------------------------
# 底图
# ---------------------------------------------------------------------------
def base_image(grid_map: GridMap) -> np.ndarray:
    """生成 RGB 底图数组 (H, W, 3) uint8（占用黑 / 空闲白 / 未知灰）。"""
    gray = grid_to_pgm_image(grid_map.grid)          # (H, W) uint8
    rgb = np.stack([gray, gray, gray], axis=-1)
    # 区分三态配色（PGM 里 205 是灰，0 是黑，254 接近白）
    rgb = np.where(gray[..., None] == 205,
                   np.array(C_UNK, dtype=np.uint8), rgb).astype(np.uint8)
    rgb = np.where(gray[..., None] == 0,
                   np.array(C_OCC, dtype=np.uint8), rgb).astype(np.uint8)
    rgb = np.where(gray[..., None] == 254,
                   np.array(C_FREE, dtype=np.uint8), rgb).astype(np.uint8)
    return rgb


def downsample(points: Sequence[Sequence[float]], max_n: int = 4000) -> List[Tuple[float, float]]:
    """轨迹抽稀：超过 max_n 时等间隔抽样，保证首尾保留。"""
    pts = [(float(p[0]), float(p[1])) for p in points if _finite2(p)]
    if max_n <= 0 or len(pts) <= max_n:
        return pts
    step = len(pts) / float(max_n)
    out: List[Tuple[float, float]] = []
    k = 0.0
    while int(k) < len(pts):
        out.append(pts[int(k)])
        k += step
    if out and out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


# ---------------------------------------------------------------------------
# 绘制
# ---------------------------------------------------------------------------
def draw_preview(grid_map: GridMap,
                 path: str,
                 boundary: Optional[np.ndarray] = None,
                 given_point: Sequence[float] = (0.0, 0.0)) -> str:
    """绘制 `map_preview.png`：三色底图 + 场地边界（蓝）+ 给定点（品红十字）。

    Args:
        grid_map: 栅格地图。
        path: 输出 PNG 路径。
        boundary: (N, 2) 边界多边形（map 系）；None 时不画。
        given_point: 给定点（map 原点）坐标。

    Returns:
        str：实际写入的文件路径。
    """
    _ensure_dir(os.path.dirname(os.path.abspath(path)))
    rgb = base_image(grid_map)
    img = Image.fromarray(rgb, mode='RGB')
    drw = ImageDraw.Draw(img)
    if boundary is not None and len(boundary) >= 2:
        pts = [world_to_pixel(p, grid_map) for p in boundary]
        pts.append(pts[0])
        drw.line(pts, fill=C_BOUNDARY, width=2)
    _draw_cross(drw, world_to_pixel(given_point, grid_map), 12, C_GIVEN, width=2)
    img.save(path, format='PNG')
    return path


def draw_tracked(grid_map: GridMap,
                 path: str,
                 target_traj: Sequence[Sequence[float]] = (),
                 target_cur: Optional[Sequence[float]] = None,
                 target_pred: Optional[Sequence[float]] = None,
                 base_traj: Sequence[Sequence[float]] = (),
                 base_cur: Optional[Sequence[float]] = None,
                 given_point: Sequence[float] = (0.0, 0.0),
                 boundary: Optional[np.ndarray] = None) -> str:
    """绘制 `map_tracked.png`：底图 + 轨迹与关键标注。

    必须标注（DESIGN §6.11）：
    目标估计轨迹(蓝)、目标当前(红点)、预测(红×)、底盘轨迹(绿)、
    底盘当前(绿方块)、给定点原点(品红十字)。

    Args:
        grid_map: 栅格地图。
        path: 输出 PNG 路径。
        target_traj: 目标估计轨迹点列 [(x, y), ...]（map 系）。
        target_cur: 目标当前位置 (x, y)；None 时取轨迹末点。
        target_pred: 目标预测位置 (x, y)；None 时不画。
        base_traj: 底盘轨迹点列 [(x, y), ...]。
        base_cur: 底盘当前位置；None 时取轨迹末点。
        given_point: 给定点（map 原点）。
        boundary: 可选边界多边形。

    Returns:
        str：实际写入的文件路径。
    """
    _ensure_dir(os.path.dirname(os.path.abspath(path)))
    rgb = base_image(grid_map)
    img = Image.fromarray(rgb, mode='RGB')
    drw = ImageDraw.Draw(img)

    if boundary is not None and len(boundary) >= 2:
        pts = [world_to_pixel(p, grid_map) for p in boundary]
        pts.append(pts[0])
        drw.line(pts, fill=C_BOUNDARY, width=2)

    # 底盘轨迹（绿）+ 底盘当前（绿方块）
    bpts = downsample(base_traj)
    if len(bpts) >= 2:
        drw.line([world_to_pixel(p, grid_map) for p in bpts],
                 fill=C_BASE_TRAJ, width=2)
    bc = base_cur if base_cur is not None else (bpts[-1] if bpts else None)
    if bc is not None and _finite2(bc):
        _draw_square(drw, world_to_pixel(bc, grid_map), 5, C_BASE_CUR, width=2)

    # 目标轨迹（蓝）+ 目标当前（红点）
    tpts = downsample(target_traj)
    if len(tpts) >= 2:
        drw.line([world_to_pixel(p, grid_map) for p in tpts],
                 fill=C_TARGET_TRAJ, width=2)
    tc = target_cur if target_cur is not None else (tpts[-1] if tpts else None)
    if tc is not None and _finite2(tc):
        _draw_dot(drw, world_to_pixel(tc, grid_map), 4, C_TARGET_CUR)

    # 预测（红×）
    if target_pred is not None and _finite2(target_pred):
        _draw_x(drw, world_to_pixel(target_pred, grid_map), 7, C_TARGET_PRED, width=2)

    # 给定点（品红十字）
    _draw_cross(drw, world_to_pixel(given_point, grid_map), 12, C_GIVEN, width=2)

    img.save(path, format='PNG')
    return path


# ---------------------------------------------------------------------------
# 图元
# ---------------------------------------------------------------------------
def _draw_cross(drw: 'ImageDraw.ImageDraw', p: Tuple[float, float], r: int,
                color: Tuple[int, int, int], width: int = 2) -> None:
    """画十字（给定点标记）。"""
    x, y = p
    drw.line([(x - r, y), (x + r, y)], fill=color, width=width)
    drw.line([(x, y - r), (x, y + r)], fill=color, width=width)


def _draw_x(drw: 'ImageDraw.ImageDraw', p: Tuple[float, float], r: int,
            color: Tuple[int, int, int], width: int = 2) -> None:
    """画 ×（预测位置标记）。"""
    x, y = p
    drw.line([(x - r, y - r), (x + r, y + r)], fill=color, width=width)
    drw.line([(x - r, y + r), (x + r, y - r)], fill=color, width=width)


def _draw_dot(drw: 'ImageDraw.ImageDraw', p: Tuple[float, float], r: int,
              color: Tuple[int, int, int]) -> None:
    """画实心圆点（目标当前位置）。"""
    x, y = p
    drw.ellipse([(x - r, y - r), (x + r, y + r)], fill=color, outline=color)


def _draw_square(drw: 'ImageDraw.ImageDraw', p: Tuple[float, float], r: int,
                 color: Tuple[int, int, int], width: int = 2) -> None:
    """画方块轮廓（底盘当前位置）。"""
    x, y = p
    drw.rectangle([(x - r, y - r), (x + r, y + r)], outline=color, width=width)


def _finite2(p: Sequence[float]) -> bool:
    """判断二维点是否有限。"""
    return bool(math.isfinite(float(p[0])) and math.isfinite(float(p[1])))


def _ensure_dir(d: str) -> None:
    """确保目录存在。"""
    if d:
        os.makedirs(d, exist_ok=True)
