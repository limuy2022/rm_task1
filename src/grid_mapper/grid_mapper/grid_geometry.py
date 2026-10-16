"""Planar OccupancyGrid metadata transforms; no ROS imports and no resampling.

An OccupancyGrid stores its cell-axis rotation in info.origin.orientation.
Changing its reference frame composes origins; relabeling header.frame_id alone
would move/rotate the map incorrectly. A tilted grid cannot be represented by
this planar implementation and is rejected rather than silently flattened.
"""
from __future__ import annotations
import math
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
