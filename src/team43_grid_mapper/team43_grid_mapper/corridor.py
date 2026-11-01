"""两侧直墙的几何近似；不估计沿走廊的位置，也不修改LIO。"""
import math

import cv2
import numpy as np


def fit_parallel_walls(points, *, resolution=.05, expected_width=None, min_length=2.):
    """从XY障碍点找有纵向重叠的平行长线段，返回法向偏移及宽度。

    不把门扇、端点或观测窗口视为赛区边界。Hough只是拟合候选，不证明实物准确。
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('墙面点必须为N×2')
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 40:
        raise ValueError('墙面点不足')
    lower = np.floor(points.min(axis=0)/resolution)*resolution-resolution
    ij = np.floor((points-lower)/resolution).astype(int)
    shape = ij.max(axis=0)+2
    if max(shape) > 1600:
        raise ValueError('拟合范围过大')
    image = np.zeros((shape[1], shape[0]), np.uint8)
    image[ij[:, 1], ij[:, 0]] = 255
    lines = cv2.HoughLinesP(image, .5, np.pi/720, 25,
                          minLineLength=int(min_length/resolution), maxLineGap=12)
    if lines is None:
        raise ValueError('没有足够长的侧墙')
    segments = []
    for a,b,c,d in lines[:,0]:
        p = (np.array([[a,b],[c,d]])+.5)*resolution+lower
        delta = p[1]-p[0]
        length = float(np.linalg.norm(delta))
        yaw = math.atan2(delta[1],delta[0]) % math.pi
        segments.append((length,yaw,p))
    segments.sort(key=lambda row:row[0], reverse=True)
    best = None
    for i,(la,ya,pa) in enumerate(segments[:80]):
        for lb,yb,pb in segments[i+1:80]:
            da = (yb-ya+math.pi/2) % math.pi-math.pi/2
            if abs(da) > math.radians(5):
                continue
            yaw = (ya+da*lb/(la+lb)) % math.pi
            u = np.array([math.cos(yaw),math.sin(yaw)])
            n = np.array([-u[1],u[0]])
            offsets = sorted([float((pa@n).mean()),float((pb@n).mean())])
            width = offsets[1]-offsets[0]
            if not 1.2 <= width <= 5.:
                continue
            if expected_width is not None and abs(width-expected_width) > .35:
                continue
            sa,sb = pa@u,pb@u
            overlap = min(sa.max(),sb.max())-max(sa.min(),sb.min())
            if overlap < min_length:
                continue
            score = overlap+.15*(la+lb)
            if expected_width is not None:
                score -= 2*abs(width-expected_width)
            if best is None or score > best[0]:
                best = (score,yaw,offsets,u,n)
    if best is None:
        raise ValueError('未找到宽度合理且重叠的两侧平行墙')
    _,yaw,offsets,u,n = best
    projected = points@n
    supports = [points[np.abs(projected-offset)<.12] for offset in offsets]
    if min(map(len,supports)) < 20:
        raise ValueError('侧墙支持点不足')
    offsets = [float(np.median(p@n)) for p in supports]
    span = np.concatenate([p@u for p in supports])
    width = offsets[1]-offsets[0]
    if expected_width is not None and abs(width-expected_width) > .35:
        raise ValueError('细化后的墙宽与上午估计不符')
    return {'yaw_rad':float(yaw), 'normal_offsets_m':offsets, 'width_m':width,
            'observed_span_m':[float(span.min()),float(span.max())],
            'support_points':[len(p) for p in supports],
            'method':'平行侧墙近似；不是全局定位或门间长度测量'}


def corridor_grid(fit, resolution=.05, length=40.):
    """有限显示窗口内的两条墙；不生成端墙，窗口长度不是测得场地长度。"""
    width = float(fit['width_m'])
    shape = (int(round(length/resolution)), int(math.ceil((width+1.)/resolution))+1)
    grid = np.full(shape,-1,np.int8)
    a = int(round(.5/resolution));b = int(round((.5+width)/resolution))
    grid[:,a+1:b] = 0
    grid[:,a] = grid[:,b] = 100
    yaw = fit['yaw_rad'];u = np.array([math.cos(yaw),math.sin(yaw)])
    n = np.array([-u[1],u[0]])
    mid = sum(fit['observed_span_m'])/2
    origin = u*(mid-length/2)+n*(fit['normal_offsets_m'][0]-.5)
    return grid,[float(origin[0]),float(origin[1]),float(yaw)]
