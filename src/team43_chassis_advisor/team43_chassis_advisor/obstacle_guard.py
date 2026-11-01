"""实时二维障碍点对建议速度的约束；不控制底盘，不依赖旧地图。"""
import math

import numpy as np


def guard_velocity(command, points, *, clearance=.4, horizon=1.5, braking=.5,
                   voxel_size=.1):
    """检查圆形车体沿候选速度扫过的区域，限速或选择邻近可行方向。

    points为相对底盘中心的障碍点。网格化后将半格对角线加入半径，
    不因离散化把原本碰撞的点挪出车体。速度下限必须在本函数之前应用。
    """
    command = np.asarray(command,dtype=float).reshape(2)
    points = np.asarray(points,dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('障碍点必须为有限N×2数组')
    if (not np.isfinite(command).all() or
            not all(math.isfinite(v) and v>0 for v in (clearance,horizon,braking,voxel_size))):
        raise ValueError('建议速度或保护参数无效')
    speed = float(np.linalg.norm(command))
    if speed < 1e-9:
        return np.zeros(2),0.,False
    if not len(points):
        # 空输入不证明通道空闲。
        return np.zeros(2),0.,True
    points = (np.unique(np.floor(points/voxel_size),axis=0)+.5)*voxel_size
    radius = clearance+voxel_size/math.sqrt(2)
    desired = command/speed
    # 只在目标前方半平面找方向；不能为了避障建议向目标反向行驶。
    angles = np.r_[0.,np.deg2rad(np.arange(1.,90.)), -np.deg2rad(np.arange(1.,90.))]
    angles = angles[np.argsort(np.abs(angles),kind='stable')]
    c,s = np.cos(angles),np.sin(angles)
    directions = np.c_[c*desired[0]-s*desired[1],s*desired[0]+c*desired[1]]
    forward = directions @ points.T
    d2 = np.sum(points*points,axis=1)
    sideways2 = np.maximum(0.,d2[None,:]-forward*forward)
    intersects = (forward>1e-9)&(sideways2<radius*radius)
    entry = forward-np.sqrt(np.maximum(0.,radius*radius-sideways2))
    distances = np.min(np.where(intersects,np.maximum(0.,entry),np.inf),axis=1)
    caps = np.minimum(distances/horizon,np.sqrt(2*braking*distances))
    speeds = np.minimum(speed,caps)
    # 优先保持原方向；若只能极慢向墙挪动，则寻找能保留大部分速度的邻近方向。
    threshold = min(speed*.8,.1)
    good = np.flatnonzero(speeds >= threshold)
    if not len(good):
        return np.zeros(2),0.,True
    k = int(good[0])
    return directions[k]*speeds[k],float(angles[k]),False
