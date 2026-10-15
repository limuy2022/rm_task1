"""gimbal_common 缺失时的本地兜底实现（**仅用于 Windows 无 ROS2 的开发环境**）。

背景（主理人拍板 2）：本机是 Windows，没有 ROS2、无法 `colcon build`，
`gimbal_common`（T01，由工程师 A 负责）可能尚未就绪。为了让本包的算法内核
（`grid_builder.py` / `pgm_io.py` / `cost_raycast.py` / `velocity_law.py` /
`csv_writer.py`）能脱离 rclpy 直接 import 并跑 pytest，这里提供与
`gimbal_common` **签名完全一致**的最小兜底实现。

在 Ubuntu 24.04 + ROS2 Jazzy 上，`gimbal_common` 已由 T01 安装，
`try: from gimbal_common...` 会成功，本模块不会被使用（可直接删除）。

本模块**不修改** `gimbal_common` 的任何文件，只是只读的降级镜像。

⚠ 已对齐工程师 A 的**真实签名**（2026-10-09 核对 `gimbal_common/math_utils.py`）：
  - ``ray_cells(origin_xy, dir_xy, L, res)`` 的 ``origin_xy`` 是**栅格局部米制坐标**
    （即已减掉栅格原点），不是世界坐标；退化输入返回 ``[]``。
  - ``accel_limit(delta_xy, dt, a_max)`` 只接受**增量**并返回**限幅后的增量**
    （不是 (cmd, prev, dt, a_max)）。调用方需自行 ``prev + 限幅增量``。
  - ``load_yaml(path, required=True)`` 在文件缺失且 required=True 时抛异常。
若 A 后续改动这些签名，请以 A 为准并同步本文件（本文件只是镜像）。
"""

import math
import os
import time
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np

__all__ = [
    'now',
    'clamp',
    'lowpass',
    'accel_limit',
    'ray_cells',
    'rotz2',
    'COLS8',
    'COLS_EXTRA',
    'format_row',
    'get_logger',
    'throttled',
    'load_yaml',
]


# ---------------------------------------------------------------------------
# time_utils
# ---------------------------------------------------------------------------
def now() -> float:
    """单调时钟（= time.monotonic()），全工程唯一时间入口。"""
    return time.monotonic()


# ---------------------------------------------------------------------------
# math_utils（签名对齐 gimbal_common）
# ---------------------------------------------------------------------------
def clamp(v: float, lo: float, hi: float) -> float:
    """把 v 裁剪到 [lo, hi]；非有限输入返回 lo。"""
    out = float(v)
    if not math.isfinite(out):
        return float(lo)
    if lo > hi:
        lo, hi = hi, lo
    return max(float(lo), min(float(hi), out))


def lowpass(prev: float, target: float, dt: float, cutoff_hz: float) -> float:
    """一阶低通：α = 1 − exp(−2π·f_c·dt)。输入非有限时回退 prev。"""
    p = float(prev)
    t = float(target)
    dt = float(dt)
    if not math.isfinite(t) or not math.isfinite(dt):
        return p if math.isfinite(p) else 0.0
    if dt <= 0.0 or cutoff_hz <= 0.0:
        return t
    alpha = 1.0 - math.exp(-2.0 * math.pi * float(cutoff_hz) * dt)
    return p + alpha * (t - p)


def accel_limit(delta_xy: Sequence[float], dt: float,
                a_max: float) -> np.ndarray:
    """对二维**增量**做加速度限幅（A 的签名，返回限幅后的增量）。

    调用方应写 `cmd = prev + accel_limit(cmd_target - prev, dt, a_max)`。
    """
    d = np.asarray(delta_xy, dtype=np.float64).reshape(2)
    if not np.all(np.isfinite(d)):
        return np.zeros(2, dtype=np.float64)
    dt = float(dt)
    a_max = float(a_max)
    if a_max <= 0.0 or dt <= 0.0:
        return d
    n = float(np.linalg.norm(d))
    max_step = a_max * dt
    if n <= max_step or n <= 0.0:
        return d
    return d * (max_step / n)


def rotz2(theta: float) -> np.ndarray:
    """二维绕 +z 旋转矩阵 (2,2)。"""
    c = math.cos(float(theta))
    s = math.sin(float(theta))
    return np.array([[c, -s], [s, c]], dtype=float)


def ray_cells(origin_xy: Sequence[float],
              dir_xy: Sequence[float],
              L: float,
              res: float) -> List[Tuple[int, int]]:
    """DDA（Amanatides-Woo）网格遍历，返回被射线穿过的 cell 索引 [(i, j), ...]。

    ⚠ **坐标约定（与 gimbal_common 一致）**：`origin_xy` 是**栅格局部米制坐标**
    （世界坐标 − 栅格原点），cell 索引 = `floor(local / res)`。
    退化输入（零方向 / L≤0 / res≤0）返回空列表。

    采用 **超覆盖（supercover）**：正穿格点时同时补上两个侧向邻格，避免"穿角漏检"。
    返回列表按由近及远顺序，起点 cell 在最前；不保证唯一（调用方按需去重）。

    Args:
        origin_xy: 射线起点（**栅格局部米**）。
        dir_xy: 射线方向（无需归一化）。
        L: 射线长度（米）。
        res: 栅格分辨率（米/格）。

    Returns:
        list[tuple[int, int]]：(i, j) cell 索引列表。
    """
    res = float(res)
    L = float(L)
    if res <= 0.0 or L <= 0.0:
        return []
    ox = float(origin_xy[0])
    oy = float(origin_xy[1])
    dx = float(dir_xy[0])
    dy = float(dir_xy[1])
    n = math.hypot(dx, dy)
    if n <= 1e-12:
        return []
    dx /= n
    dy /= n

    x = ox / res                 # 连续栅格坐标（1 单位 = 1 cell）
    y = oy / res
    t_end = L / res
    ix = int(math.floor(x))
    iy = int(math.floor(y))

    step_x = 1 if dx > 0.0 else (-1 if dx < 0.0 else 0)
    step_y = 1 if dy > 0.0 else (-1 if dy < 0.0 else 0)

    if step_x > 0:
        t_max_x = (math.floor(x) + 1.0 - x) / dx
    elif step_x < 0:
        t_max_x = (x - math.floor(x)) / (-dx)
    else:
        t_max_x = math.inf
    if step_y > 0:
        t_max_y = (math.floor(y) + 1.0 - y) / dy
    elif step_y < 0:
        t_max_y = (y - math.floor(y)) / (-dy)
    else:
        t_max_y = math.inf
    t_delta_x = abs(1.0 / dx) if dx != 0.0 else math.inf
    t_delta_y = abs(1.0 / dy) if dy != 0.0 else math.inf

    cells: List[Tuple[int, int]] = [(ix, iy)]
    t = 0.0
    guard = 0
    max_iter = int(4.0 * (abs(math.floor(x + dx * t_end) - ix)
                          + abs(math.floor(y + dy * t_end) - iy))) + 64
    while t <= t_end and guard <= max_iter:
        guard += 1
        if t_max_x < t_max_y:
            ix += step_x
            t = t_max_x
            t_max_x += t_delta_x
        elif t_max_y < t_max_x:
            iy += step_y
            t = t_max_y
            t_max_y += t_delta_y
        else:
            # 正好穿过格点：超覆盖，把两个侧向邻格一并加入
            jx = ix + step_x
            jy = iy + step_y
            if jx != ix:
                cells.append((jx, iy))
            if jy != iy:
                cells.append((ix, jy))
            ix, iy = jx, jy
            t = t_max_x
            t_max_x += t_delta_x
            t_max_y += t_delta_y
        if t > t_end:
            break
        cells.append((ix, iy))
    return cells


# ---------------------------------------------------------------------------
# csv_utils
# ---------------------------------------------------------------------------
COLS8: List[str] = ['pred_x', 'pred_y', 'cur_x', 'cur_y',
                    'base_x', 'base_y', 'cmd_vx', 'cmd_vy']
COLS_EXTRA: List[str] = ['timestamp', 'track_status']


def format_row(vals: Sequence[float], extra: bool = False) -> str:
    """把一行数值格式化为 CSV 文本（严格 8 列；extra=True 时追加第 9/10 列）。"""
    line = ','.join('%.6f' % float(v) for v in list(vals)[:8])
    if extra:
        tail = list(vals)[8:10]
        ts = float(tail[0]) if len(tail) > 0 else 0.0
        st = int(tail[1]) if len(tail) > 1 else 0
        line += ',%.6f,%d' % (ts, st)
    return line


def header_line(extra: bool = False) -> str:
    """表头行（列数与 format_row 一致）。"""
    cols = list(COLS8) + (list(COLS_EXTRA) if extra else [])
    return ','.join(cols)


# ---------------------------------------------------------------------------
# log_utils
# ---------------------------------------------------------------------------
class _NullLogger:
    """无 ROS 环境下的极简 logger（接口与 rclpy logger 的常用子集对齐）。"""

    def __init__(self, name: str = 'null') -> None:
        self._name = str(name)

    def _emit(self, level: str, msg: str) -> None:
        print('[%s] %s %s' % (self._name, level, msg))

    def debug(self, msg: str) -> None:
        self._emit('DEBUG', msg)

    def info(self, msg: str) -> None:
        self._emit('INFO', msg)

    def warn(self, msg: str) -> None:
        self._emit('WARN', msg)

    def error(self, msg: str) -> None:
        self._emit('ERROR', msg)

    def fatal(self, msg: str) -> None:
        self._emit('FATAL', msg)


_THROTTLE_STATE: Dict[str, float] = {}


def get_logger(node: Any = None, name: str = 'node') -> Any:
    """取 logger。有 ROS 节点时用节点的 logger；否则返回极简终端 logger。"""
    if node is not None and hasattr(node, 'get_logger'):
        try:
            return node.get_logger()
        except Exception:  # pragma: no cover - 兜底，避免打日志打断业务
            return _NullLogger(name)
    return _NullLogger(name)


def throttled(logger: Any, level: str, msg: str, key: str,
              period_s: float = 1.0) -> bool:
    """节流日志：同一 key 在 period_s 内只打一次。返回本次是否真的打了。"""
    t = now()
    last = _THROTTLE_STATE.get(key, -1e18)
    if t - last < float(period_s):
        return False
    _THROTTLE_STATE[key] = t
    fn = getattr(logger, str(level).lower(), None)
    if fn is None:
        fn = getattr(logger, 'info', None)
    if fn is not None:
        fn(msg)
    return True


# ---------------------------------------------------------------------------
# config_loader
# ---------------------------------------------------------------------------
def load_yaml(path: str, required: bool = True) -> Optional[Dict[str, Any]]:
    """加载 YAML 文件并返回 dict。

    Args:
        path: 文件路径。
        required: 兼容 gimbal_common 的入参；本兜底实现**永不抛异常**，
            文件缺失 / 解析失败一律返回 None（调用方决定 FATAL 或默认值）。

    Returns:
        dict | None：解析结果；失败为 None。
    """
    _ = required
    try:
        import yaml  # 延迟导入，避免无 PyYAML 时本模块不可用
    except ImportError:  # pragma: no cover
        return None
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except Exception:  # pragma: no cover
        return None
    if data is None:
        return {}
    if not isinstance(data, dict):
        return None
    return data
