"""写 `map.pgm`(P5) 与 `map.yaml`，以及读回自检。

纯 Python（不依赖 rclpy），可直接单测。规格见 DESIGN §6.11。

⚠⚠ **PGM 行序必须翻转**：PGM 的第 0 行是图像**顶行**，对应 **y 最大**。
写文件时必须：

```python
for r in range(H - 1, -1, -1):        # r = H-1-j  （r 是图像行号）
    row = grid[:, (H - 1 - r)]        # j = H-1-r  （j 是 y 方向 cell 索引）
    f.write(row.tobytes())
```

`map.yaml` 标准格式：

```yaml
image: map.pgm
mode: trinary
resolution: 0.05
origin: [-6.0, -6.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```
"""

from __future__ import annotations

import os
from typing import Dict
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np

from .grid_builder import (FREE, OCCUPIED, UNKNOWN, PGM_FREE, PGM_OCCUPIED,
                           PGM_UNKNOWN, GridMap)

__all__ = [
    'PGM_UNKNOWN',
    'PGM_FREE',
    'PGM_OCCUPIED',
    'grid_to_pgm_bytes',
    'grid_to_pgm_image',
    'pgm_to_grid',
    'write_pgm',
    'read_pgm',
    'read_grid',
    'write_map_yaml',
    'write_map',
]

# 默认 nav2 / map_server 参数（DESIGN §6.11）
DEFAULT_OCCUPIED_THRESH: float = 0.65
DEFAULT_FREE_THRESH: float = 0.196
DEFAULT_MODE: str = 'trinary'
DEFAULT_NEGATE: int = 0


# ---------------------------------------------------------------------------
# 写
# ---------------------------------------------------------------------------
def grid_to_pgm_image(grid: np.ndarray) -> np.ndarray:
    """把 (W, H) 三态栅格转成 PGM 图像数组 (H, W) uint8（**已翻转行序**）。

    第 0 行 = 图像顶行 = y 最大（j = H-1）。

    Args:
        grid: (W, H) int8 数组，取值 {-1, 0, 100}。

    Returns:
        np.ndarray：(H, W) uint8，取值 {0, 254, 205}。
    """
    g = np.asarray(grid)
    if g.ndim != 2:
        raise ValueError('grid must be 2-D, got shape %r' % (g.shape,))
    w, h = g.shape[0], g.shape[1]
    vals = np.where(g == OCCUPIED, PGM_OCCUPIED,
                    np.where(g == FREE, PGM_FREE, PGM_UNKNOWN)).astype(np.uint8)
    img = np.empty((h, w), dtype=np.uint8)
    for r in range(h):                    # r 是图像行号（0 = 顶行）
        img[r, :] = vals[:, (h - 1 - r)]  # j = h-1-r，顶行取 j=h-1（y 最大）
    return img


def grid_to_pgm_bytes(grid: np.ndarray) -> bytes:
    """把栅格转成 PGM P5 的**数据区**字节流（行序已翻转）。"""
    img = grid_to_pgm_image(grid)
    # 与 DESIGN §6.11 的写法等价：for r in range(H-1,-1,-1): row = grid[:, H-1-r]
    return img.tobytes()


def write_pgm(path: str, grid: np.ndarray) -> str:
    """写 P5 灰度 PGM 文件。

    Args:
        path: 目标文件路径（如 `maps/map.pgm`）。
        grid: (W, H) int8 三态数组。

    Returns:
        str：实际写入的文件路径。
    """
    _ensure_dir(os.path.dirname(os.path.abspath(path)))
    w, h = int(np.asarray(grid).shape[0]), int(np.asarray(grid).shape[1])
    with open(path, 'wb') as f:
        f.write(b'P5\n')
        f.write(('%d %d\n' % (w, h)).encode('ascii'))
        f.write(b'255\n')
        f.write(grid_to_pgm_bytes(grid))
    return path


def write_map_yaml(path: str,
                   image: str = 'map.pgm',
                   resolution: float = 0.05,
                   origin_xyz: Sequence[float] = (-6.0, -6.0, 0.0),
                   negate: int = DEFAULT_NEGATE,
                   occupied_thresh: float = DEFAULT_OCCUPIED_THRESH,
                   free_thresh: float = DEFAULT_FREE_THRESH,
                   mode: str = DEFAULT_MODE) -> str:
    """写 `map.yaml`（nav2_map_server 可加载的标准格式）。

    Args:
        path: 目标文件路径（如 `maps/map.yaml`）。
        image: PGM 文件名（与 yaml 同目录）。
        resolution: m/cell。
        origin_xyz: [x, y, yaw]，栅格左下角在世界(map)中的位姿。
        negate: 0（默认）。
        occupied_thresh: 占用阈值（0.65）。
        free_thresh: 空闲阈值（0.196）。
        mode: trinary / scale / raw。

    Returns:
        str：实际写入的文件路径。
    """
    _ensure_dir(os.path.dirname(os.path.abspath(path)))
    org = [float(origin_xyz[0]), float(origin_xyz[1]),
           float(origin_xyz[2]) if len(origin_xyz) > 2 else 0.0]
    lines = [
        'image: %s' % str(image),
        'mode: %s' % str(mode),
        'resolution: %s' % _fmt_num(float(resolution)),
        'origin: [%s, %s, %s]' % (_fmt_num(org[0]), _fmt_num(org[1]), _fmt_num(org[2])),
        'negate: %d' % int(negate),
        'occupied_thresh: %s' % _fmt_num(float(occupied_thresh)),
        'free_thresh: %s' % _fmt_num(float(free_thresh)),
    ]
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    return path


def write_map(out_dir: str,
              grid_map: GridMap,
              image_name: str = 'map.pgm',
              yaml_name: str = 'map.yaml',
              origin_xyz: Optional[Sequence[float]] = None,
              occupied_thresh: float = DEFAULT_OCCUPIED_THRESH,
              free_thresh: float = DEFAULT_FREE_THRESH) -> Dict[str, str]:
    """一次写出 `map.pgm` + `map.yaml`。

    Args:
        out_dir: 输出目录（如 `maps/`）。
        grid_map: 栅格地图（写的是**干净三态** `grid`，不含膨胀）。
        image_name: PGM 文件名。
        yaml_name: YAML 文件名。
        origin_xyz: 覆盖 map.yaml 的 origin；None 时用 grid_map.origin_xy 和 origin_yaw。
        occupied_thresh / free_thresh: 阈值。

    Returns:
        dict：{'pgm': 路径, 'yaml': 路径}。
    """
    _ensure_dir(out_dir)
    pgm_path = os.path.join(out_dir, image_name)
    yaml_path = os.path.join(out_dir, yaml_name)
    write_pgm(pgm_path, grid_map.grid)
    if origin_xyz is None:
        origin_xyz = (float(grid_map.origin_xy[0]),
                      float(grid_map.origin_xy[1]), float(grid_map.origin_yaw))
    write_map_yaml(yaml_path,
                   image=image_name,
                   resolution=grid_map.resolution,
                   origin_xyz=origin_xyz,
                   occupied_thresh=occupied_thresh,
                   free_thresh=free_thresh)
    return {'pgm': pgm_path, 'yaml': yaml_path}


# ---------------------------------------------------------------------------
# 读（自检用）
# ---------------------------------------------------------------------------
def read_pgm(path: str) -> Tuple[np.ndarray, int]:
    """读 P5（也兼容 P2）PGM，返回 (图像数组 (H, W) uint8, maxval)。

    Args:
        path: PGM 文件路径。

    Returns:
        tuple：(np.ndarray (H, W) uint8 按**图像行序**，int maxval)。

    Raises:
        ValueError: 文件不是合法 PGM / 数据长度不足。
        FileNotFoundError: 文件不存在。
    """
    with open(path, 'rb') as f:
        raw = f.read()
    if not raw:
        raise ValueError('empty pgm file: %s' % path)
    pos = 0
    tokens: list = []

    def _next_token() -> bytes:
        nonlocal pos
        while True:
            if pos >= len(raw):
                raise ValueError('truncated pgm header: %s' % path)
            c = raw[pos:pos + 1]
            if c == b'#':
                nl = raw.find(b'\n', pos)
                if nl < 0:
                    raise ValueError('truncated pgm comment: %s' % path)
                pos = nl + 1
                continue
            if c.isspace():
                pos += 1
                continue
            start = pos
            while pos < len(raw) and not raw[pos:pos + 1].isspace():
                pos += 1
            return raw[start:pos]

    magic = _next_token()
    if magic not in (b'P5', b'P2'):
        raise ValueError('unsupported pgm magic %r in %s' % (magic, path))
    width = int(_next_token())
    height = int(_next_token())
    maxval = int(_next_token())
    if width <= 0 or height <= 0:
        raise ValueError('bad pgm size %dx%d in %s' % (width, height, path))
    # 头结束后恰有一个空白字符
    if pos < len(raw) and raw[pos:pos + 1].isspace():
        pos += 1

    if magic == b'P5':
        if maxval > 255:
            raise ValueError('16-bit pgm not supported (maxval=%d)' % maxval)
        need = width * height
        data = raw[pos:pos + need]
        if len(data) < need:
            raise ValueError('truncated pgm data: need %d, got %d' % (need, len(data)))
        img = np.frombuffer(data, dtype=np.uint8).reshape((height, width)).copy()
    else:
        nums = raw[pos:].split()
        if len(nums) < width * height:
            raise ValueError('truncated ascii pgm data')
        img = np.array([int(v) for v in nums[:width * height]],
                       dtype=np.uint8).reshape((height, width))
    return img, maxval


def pgm_to_grid(img: np.ndarray,
                occupied_thresh: float = DEFAULT_OCCUPIED_THRESH,
                free_thresh: float = DEFAULT_FREE_THRESH,
                negate: int = DEFAULT_NEGATE,
                maxval: int = 255) -> np.ndarray:
    """PGM 图像数组 (H, W) -> 本包内部三态栅格 (W, H) int8。

    与 `nav2_map_server` 的反演一致（`negate=0`）：
    `occ = (maxval - pixel) / maxval`；
    `occ > occupied_thresh → 100`；`occ < free_thresh → 0`；其余 → -1。
    验证：pixel=0 → occ=1.0 → 100；pixel=254 → occ≈0.004 → 0；
    pixel=205 → occ≈0.1961（不小于 0.196）→ -1 ✓

    Args:
        img: (H, W) uint8 图像数组（**图像行序**，第 0 行 = y 最大）。
        occupied_thresh: 占用阈值。
        free_thresh: 空闲阈值。
        negate: 1 时反相。
        maxval: PGM 的最大灰度（通常 255）。

    Returns:
        np.ndarray：(W, H) int8 三态栅格（行序已翻转回 [i, j] 排布）。
    """
    a = np.asarray(img, dtype=float)
    h, w = a.shape[0], a.shape[1]
    mv = float(maxval) if maxval > 0 else 255.0
    if int(negate) != 0:
        occ = a / mv
    else:
        occ = (mv - a) / mv
    tri = np.where(occ > occupied_thresh, OCCUPIED,
                   np.where(occ < free_thresh, FREE, UNKNOWN)).astype(np.int8)
    # 图像行 r 对应 j = h-1-r
    grid = np.empty((w, h), dtype=np.int8)
    for r in range(h):
        grid[:, (h - 1 - r)] = tri[r, :]
    return grid


def read_grid(path: str,
              resolution: float = 0.05,
              origin_xy: Optional[Sequence[float]] = None,
              occupied_thresh: float = DEFAULT_OCCUPIED_THRESH,
              free_thresh: float = DEFAULT_FREE_THRESH) -> GridMap:
    """读回 PGM 并构造 GridMap（自检用，ray_grid 与 grid 相同）。"""
    img, maxval = read_pgm(path)
    grid = pgm_to_grid(img, occupied_thresh=occupied_thresh,
                       free_thresh=free_thresh, maxval=maxval)
    if origin_xy is None:
        origin_xy = (-0.5 * grid.shape[0] * resolution,
                     -0.5 * grid.shape[1] * resolution)
    return GridMap.from_arrays(grid, resolution, origin_xy,
                               inflation_radius_m=0.0)


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _ensure_dir(d: str) -> None:
    """确保目录存在（空路径时不做任何事）。"""
    if d:
        os.makedirs(d, exist_ok=True)


def _fmt_num(v: float) -> str:
    """数字转字符串：整数去掉小数点，浮点保留 6 位有效尾数。"""
    f = float(v)
    if abs(f - round(f)) < 1e-12:
        return '%d' % int(round(f))
    s = ('%.6f' % f).rstrip('0').rstrip('.')
    return s if s else '0'
