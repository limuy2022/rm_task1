"""aim_sim —— ArUco 瞄准链路的可退化合成图像渲染器（可复用模块，非 verify 脚本）。

在 `verify_perception_offline.render_marker`（干净针孔投影 + 透视 warp）之上
补齐真实检测环境的退化维度，供 `verify_aiming_robustness.py` 等回归使用：

* 真畸变渲染：角点用 ``projectPoints(K, D)`` 投影（渲染与检测/PnP 走同一套
  畸变路径，闭环里 `undistortPoints` 的去畸变被真正执行）；
* 传感器高斯噪声（σ_px，最后叠加，模拟读出域）；
* 方向性运动模糊（线性核，长度=像素位移，方向=图像运动方向）；
* 离焦（高斯核 σ）；
* 部分遮挡（暗矩形盖住码的一角，面积占比可控）；
* 曝光摆动（增益/偏置）；
* 背景纹理（渐变 + 低频色块 + 微噪，替代纯灰画布）。

全部退化由 ``np.random.default_rng(seed)`` 驱动，同 seed 结果确定。
需要 ROS 2 Humble 容器环境（cv2/numpy 来自 apt，与 NUC 同源）。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT / 'src' / 'gimbal_perception') not in sys.path:
    sys.path.insert(0, str(_ROOT / 'src' / 'gimbal_perception'))

from gimbal_perception.pnp_solver import marker_object_points, rpy_to_R  # noqa: E402

_MARKER_SRC = 256          # 生成的码图边长（像素）
_MARKER_PAD = 64           # 码图四周的白边（quiet zone），保证纹理背景下可检测
_SQUARE = 0.10             # 码物理边长（米），与 tracker_params 一致

# 物点/姿态与生产 PnP 同源：TL→TR→BR→BL（marker_object_points），
# 默认旋转 rpy(pi,0,0) 让码在光学系（y 向下）里正立——同 verify_perception_offline。
_OBJECT = marker_object_points(_SQUARE)
_RVEC_DEFAULT = cv2.Rodrigues(rpy_to_R(math.pi, 0.0, 0.0))[0].reshape(3)


def tilted_rvec(droll: float, dpitch: float, dyaw: float) -> np.ndarray:
    """在正立基准（rpy(pi,0,0)）上叠加小倾斜的 rvec。

    直接把小角度 rvec 传给 `render_scene` 会**替换**正立基准，码在图里
    180° 翻转、解出不同 ID——斜置姿态必须用本函数构造。
    """
    R = rpy_to_R(math.pi, 0.0, 0.0) @ rpy_to_R(float(droll), float(dpitch),
                                               float(dyaw))
    return cv2.Rodrigues(R)[0].reshape(3)

_DICT = None


def _dictionary():
    global _DICT
    if _DICT is None:
        _DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return _DICT


def _marker_image(marker_id: int) -> np.ndarray:
    """生成带白边（quiet zone）的码图。"""
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(_dictionary(), marker_id, _MARKER_SRC)
    else:
        marker = cv2.aruco.drawMarker(_dictionary(), marker_id, _MARKER_SRC)
    side = _MARKER_SRC + 2 * _MARKER_PAD
    padded = np.full((side, side), 255, np.uint8)
    padded[_MARKER_PAD:_MARKER_PAD + _MARKER_SRC,
           _MARKER_PAD:_MARKER_PAD + _MARKER_SRC] = marker
    return padded


def _camera_matrices(camera: Dict) -> Tuple[np.ndarray, np.ndarray]:
    k = np.array([[camera['fx'], 0., camera['cx']],
                  [0., camera['fy'], camera['cy']], [0., 0., 1.]])
    d = np.asarray(camera['distortion'], dtype=float)
    return k, d


def project_corners(camera: Dict, tvec, rvec=None) -> np.ndarray:
    """码的 4 外角点（物理 0.10m 方块）投影到像素（含畸变）。

    默认姿态与 `verify_perception_offline.render_marker` 完全一致
    （marker_object_points + rpy(pi,0,0)，码正立），保证 PnP 误差
    以真值 tvec 为基准可比。
    """
    if rvec is None:
        rvec = _RVEC_DEFAULT
    k, d = _camera_matrices(camera)
    pts, _ = cv2.projectPoints(_OBJECT, np.asarray(rvec, dtype=float),
                               np.asarray(tvec, dtype=float), k, d)
    return pts.reshape(4, 2)


def _texture_background(width: int, height: int, rng: np.random.Generator) -> np.ndarray:
    """渐变 + 低频色块 + 微噪的背景（替代纯灰画布）。"""
    xs = np.linspace(0., 1., width, dtype=float)
    ys = np.linspace(0., 1., height, dtype=float)
    gx, gy = np.meshgrid(xs, ys)
    base = 150. + 35. * gx + 18. * np.sin(6. * gx + 2. * gy) + 10. * gy
    blotch = rng.normal(0., 1., (6, 8))
    blotch = cv2.resize(blotch, (width, height), interpolation=cv2.INTER_CUBIC)
    img = base + 22. * blotch + rng.normal(0., 2.0, (height, width))
    return np.clip(img, 0., 255.).astype(np.uint8)


def _motion_blur(image: np.ndarray, length_px: float, direction_deg: float) -> np.ndarray:
    """沿给定方向的线性运动模糊核。"""
    length = max(1, int(round(float(length_px))))
    if length <= 1:
        return image
    kernel = np.zeros((length, length), np.float32)
    kernel[length // 2, :] = 1.0 / length
    angle = math.radians(float(direction_deg))
    rot = cv2.getRotationMatrix2D((length / 2.0 - 0.5, length / 2.0 - 0.5),
                                  math.degrees(angle), 1.0)
    kernel = cv2.warpAffine(kernel, rot, (length, length))
    return cv2.filter2D(image, -1, kernel)


def _occlude(image: np.ndarray, corners: np.ndarray, frac: float,
             rng: np.random.Generator,
             side: Optional[int] = None, u: Optional[float] = None) -> np.ndarray:
    """暗遮挡块：贴着码投影的一条边，从**外向内**咬入 ``frac × 码边长`` 的深度。

    沿该边覆盖约 60% 边长并向外盖住部分 quiet zone。
    ``side``(0上/1右/2下/3左) 与 ``u``(沿边位置 0~1) 可显式给定（回归里
    用确定性锚点消除统计方差）；缺省由 rng 抽取。
    实测语义（OpenCV 4.5.4 / DICT_4X4_50）：咬入一旦碰及码本体，
    检测大概率失败——frac 是“侵入深度”而非面积占比。
    """
    top_left = corners.min(axis=0)
    size = np.maximum(corners.max(axis=0) - top_left, 4.0)
    if side is None:
        side = int(rng.integers(0, 4))
    if u is None:
        u = float(rng.uniform(0.15, 0.35))
    horizontal = side % 2 == 0
    bite_dim = size[1] if horizontal else size[0]   # 咬入深度的方向
    span_dim = size[0] if horizontal else size[1]   # 沿边覆盖长度
    bite = float(frac) * float(bite_dim)
    # frac=0 表示只吞 quiet zone：终点留在码边外 2px，不碰码本体
    # （实测贴边的暗块会拉偏 cornerSubPix，使 PnP 重投影超标被拒）。
    gap = 2.0 if bite <= 0.0 else 0.0
    bite_in = max(1.0, bite + gap) - gap            # 实际深入码内的长度
    outside = 0.6 * float(_MARKER_PAD) / _MARKER_SRC * float(bite_dim)
    span = 0.6 * float(span_dim)
    dark = int(rng.integers(35, 75))
    h, w = image.shape[:2]
    if horizontal:
        x0 = top_left[0] + u * size[0]
        y0 = top_left[1] - outside if side == 0 else top_left[1] + size[1] - bite_in
        x1, y1 = x0 + span, y0 + outside + bite_in
    else:
        y0 = top_left[1] + u * size[1]
        x0 = top_left[0] - outside if side == 3 else top_left[0] + size[0] - bite_in
        x1, y1 = x0 + outside + bite_in, y0 + span
    xi0, yi0 = int(np.clip(round(x0), 0, w - 2)), int(np.clip(round(y0), 0, h - 2))
    xi1, yi1 = int(min(w, round(x1))), int(min(h, round(y1)))
    if xi1 > xi0 and yi1 > yi0:
        image[yi0:yi1, xi0:xi1] = dark
    return image


def render_scene(camera: Dict, marker_id: int, tvec, rvec=None, *,
                 rng: Optional[np.random.Generator] = None,
                 noise_sigma_px: float = 0.0,
                 motion_blur_px: float = 0.0, blur_dir_deg: float = 0.0,
                 defocus_px: float = 0.0,
                 occlusion_frac: Optional[float] = None,
                 occlusion_side: Optional[int] = None,
                 occlusion_u: Optional[float] = None,
                 gain: float = 1.0, bias: float = 0.0,
                 background: str = 'flat') -> Tuple[np.ndarray, np.ndarray]:
    """渲染一帧可退化的合成 ArUco 场景。

    Args:
        camera: 内参字典（width/height/fx/fy/cx/cy/distortion），
            通常来自 `camera_team47.yaml`。
        marker_id: ArUco ID（DICT_4X4_50，0..49）。
        tvec/rvec: 码在相机光学系下的位姿（rvec=None 表示生产基准姿态）。
        rng: 种子发生器（确定性；None 时退化为干净渲染的等价路径）。
        noise_sigma_px: 传感器高斯噪声 σ（像素灰度）。
        motion_blur_px / blur_dir_deg: 运动模糊长度（像素）与方向。
        defocus_px: 离焦高斯 σ。
        occlusion_frac: 遮挡从码边**向内咬入**的深度占码边长之比。
            None=不遮挡；0.0=只吞 quiet zone；碰及码本体后检测
            大概率失败（OpenCV 4.5.4 实测）。
        occlusion_side / occlusion_u: 遮挡锚定的边（0上/1右/2下/3左）与
            沿边位置（0~1）；显式给定可消除随机性。
        gain / bias: 曝光增益与偏置。
        background: 'flat'（纯白，等价旧渲染）或 'texture'。

    Returns:
        (image, corners)：uint8 灰度图与**真值角点像素**（4×2，含畸变投影）。
    """
    if rng is None:
        rng = np.random.default_rng(0)
    width, height = int(camera['width']), int(camera['height'])
    corners = project_corners(camera, tvec, rvec)

    if background == 'texture':
        image = _texture_background(width, height, rng).astype(np.float32)
    else:
        image = np.full((height, width), 255.0, np.float32)

    # 透视贴图：src 取**码区**（去掉白边）四角，物理角点与生产
    # marker_object_points 同序；白边经同一单应自然落在码四周作 quiet zone。
    marker = _marker_image(marker_id)
    p = _MARKER_PAD
    src = np.array([[float(p), float(p)],
                    [float(p + _MARKER_SRC - 1), float(p)],
                    [float(p + _MARKER_SRC - 1), float(p + _MARKER_SRC - 1)],
                    [float(p), float(p + _MARKER_SRC - 1)]], np.float32)
    warp = cv2.getPerspectiveTransform(src, corners.astype(np.float32))
    warped = cv2.warpPerspective(marker, warp, (width, height),
                                 flags=cv2.INTER_NEAREST)
    mask = cv2.warpPerspective(np.full(marker.shape, 255, np.uint8), warp,
                               (width, height), flags=cv2.INTER_NEAREST)
    image[mask > 0] = warped[mask > 0].astype(np.float32)

    if occlusion_frac is not None:
        image = _occlude(image.astype(np.uint8), corners, float(occlusion_frac),
                         rng, side=occlusion_side, u=occlusion_u).astype(np.float32)
    if motion_blur_px > 0.0:
        image = _motion_blur(image.astype(np.uint8), motion_blur_px,
                             blur_dir_deg).astype(np.float32)
    if defocus_px > 0.0:
        image = cv2.GaussianBlur(image, (0, 0), float(defocus_px))
    if gain != 1.0 or bias != 0.0:
        image = image * float(gain) + float(bias)
    if noise_sigma_px > 0.0:
        image = image + rng.normal(0.0, float(noise_sigma_px), image.shape)
    return np.clip(image, 0., 255.).astype(np.uint8), corners
