"""Planar OccupancyGrid metadata transforms; no ROS imports and no resampling.

An OccupancyGrid stores its cell-axis rotation in info.origin.orientation.
Changing its reference frame composes origins; relabeling header.frame_id alone
would move/rotate the map incorrectly. A tilted grid cannot be represented by
this planar implementation and is rejected rather than silently flattened.
"""
from __future__ import annotations
import math
from typing import Any, List, Tuple

import numpy as np


def planar_quaternion_yaw(xyzw) -> float:
    q = np.asarray(xyzw, dtype=float).reshape(4)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or norm < 1e-12:
        raise ValueError('map quaternion must be finite and nonzero')
    x, y, z, w = q/norm
    # A 2D map's positive z must agree with reference positive z.
    z_axis = np.array([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)])
    if not np.allclose(z_axis, [0., 0., 1.], atol=1e-6):
        raise ValueError('tilted OccupancyGrid/TF cannot be flattened into a 2D map')
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def compose_grid_origin(origin_xy, origin_yaw: float,
                        translation_xyz=(0., 0., 0.), rotation_xyzw=(0., 0., 0., 1.)):
    """T_target_grid = T_target_source @ T_source_grid, without modifying cells."""
    translation = np.asarray(translation_xyz, dtype=float).reshape(3)
    xy = np.asarray(origin_xy, dtype=float).reshape(2)
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(xy)) or not math.isfinite(origin_yaw):
        raise ValueError('map origin and transform must be finite')
    yaw = planar_quaternion_yaw(rotation_xyzw)
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.array([[c, -s], [s, c]]) @ xy + translation[:2]
    angle = math.atan2(math.sin(yaw+origin_yaw), math.cos(yaw+origin_yaw))
    return out, angle


def resolve_map_origin(grid_map: Any,
                       params_origin_xyz: Any) -> Tuple[List[float], str]:
    """决定写进 `map.yaml` 的 origin，并给出不一致时的告警文本。

    **必须以栅格自身的 `origin_xy` 为准**，不能套 `grid.origin_xyz`：

    * `map_source=config`：栅格 origin 来自 `field.yaml`，可能与 `grid.origin_xyz` 不同；
    * `map_source=topic`：栅格 origin 是**外部 SLAM 地图自带**的，
      与 `grid.origin_xyz`（12×12m 窗口那个 -6,-6）毫无关系——套错会让整张图平移。

    Args:
        grid_map: `GridMap`（用其 `origin_xy`，鸭子类型，无需 ROS）。
        params_origin_xyz: 参数里的 `grid.origin_xyz`（仅用于比对告警）。

    Returns:
        (origin_xyz, warning)：`origin_xyz` 为 `[x, y, yaw]`；一致时 `warning` 为空串。
    """
    try:
        got = [float(grid_map.origin_xy[0]), float(grid_map.origin_xy[1]), float(getattr(grid_map, 'origin_yaw', 0.0))]
    except Exception:  # pragma: no cover - 兜底
        return [0.0, 0.0, 0.0], ''
    try:
        want = [float(params_origin_xyz[0]), float(params_origin_xyz[1]),
                float(params_origin_xyz[2]) if len(params_origin_xyz) > 2 else 0.0]
    except Exception:
        return got, ''
    if any(abs(got[i]-want[i]) > 1e-9 for i in range(3)):
        return got, ('grid.origin_xyz=%s 与栅格实际 origin=%s 不一致'
                     '（map_source=topic 时外部地图自带 origin）→ map.yaml 按栅格 origin 写'
                     % (want, got))
    return got, ''
