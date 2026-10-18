"""`run_meta.json`：参数快照 + 版本信息 + 产物清单。

纯 Python（不依赖 rclpy）。

⚠ 时钟约定（DESIGN §8.2）：全工程**只有 run_id 目录命名**允许用系统时钟。
`run_meta.json` 里的时间一律记**单调时钟**基准（`t0_monotonic` / `duration_s`），
不使用 `time.time()` / `datetime.now()`，避免系统时钟跳变污染产物。
"""

from __future__ import annotations

import json
import os
import platform
import sys
from typing import Any
from typing import Dict
from typing import List
from typing import Optional

from gimbal_common.time_utils import now

__all__ = ['RunMeta', 'DEFAULT_META_NAME']

DEFAULT_META_NAME: str = 'run_meta.json'


class RunMeta:
    """一次运行的元数据收集器。

    Attributes:
        run_id: 运行标识（默认 `latest`）。
        out_dir: 输出根目录（默认 `outputs`）。
        t0_monotonic: 起始单调时刻（s），构造时记录。
    """

    def __init__(self, run_id: str = 'latest', out_dir: str = 'outputs',
                 t0_monotonic: Optional[float] = None) -> None:
        self.run_id = str(run_id)
        self.out_dir = str(out_dir)
        self.t0_monotonic = float(t0_monotonic) if t0_monotonic is not None else now()
        self._params: Dict[str, Any] = {}
        self._artifacts: List[Dict[str, Any]] = []
        self._extra: Dict[str, Any] = {}

    # -- 目录 -----------------------------------------------------------------
    @property
    def run_dir(self) -> str:
        """本次运行的输出目录 `outputs/<run_id>`。"""
        return os.path.join(self.out_dir, self.run_id)

    # -- 内容填充 -------------------------------------------------------------
    def set_params(self, params: Dict[str, Any]) -> None:
        """记录参数快照（覆盖同名 key）。"""
        if isinstance(params, dict):
            for k, v in params.items():
                self._params[str(k)] = _jsonable(v)

    def add_artifact(self, name: str, path: str) -> Dict[str, Any]:
        """登记一个产物文件（自动记录存在性与字节数）。"""
        item: Dict[str, Any] = {
            'name': str(name),
            'path': str(path),
            'exists': False,
            'bytes': 0,
        }
        try:
            if path and os.path.isfile(path):
                item['exists'] = True
                item['bytes'] = int(os.path.getsize(path))
        except OSError:  # pragma: no cover
            pass
        self._artifacts.append(item)
        return item

    def set_extra(self, key: str, value: Any) -> None:
        """附加自定义字段（如 csv 统计、blocked 次数等）。"""
        self._extra[str(key)] = _jsonable(value)

    # -- 序列化 ---------------------------------------------------------------
    def to_dict(self) -> Dict[str, Any]:
        """组装成可 JSON 序列化的 dict。"""
        return {
            'run_id': self.run_id,
            'out_dir': self.out_dir,
            'run_dir': self.run_dir,
            't0_monotonic': self.t0_monotonic,
            'duration_s': max(0.0, now() - self.t0_monotonic),
            'versions': {
                'python': platform.python_version(),
                'platform': platform.platform(),
                'numpy': _numpy_version(),
                'csvwriter': '1.0',
            },
            'params': dict(self._params),
            'artifacts': list(self._artifacts),
            'extra': dict(self._extra),
        }

    def write(self, path: Optional[str] = None) -> str:
        """写出 `run_meta.json`。

        Args:
            path: 目标路径；None 时为 `<out_dir>/<run_id>/run_meta.json`。

        Returns:
            str：实际写出的路径（失败时仍返回该路径）。
        """
        if path is None:
            path = os.path.join(self.run_dir, DEFAULT_META_NAME)
        _ensure_dir(os.path.dirname(os.path.abspath(path)))
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(self.to_dict(), f, ensure_ascii=False, indent=2,
                          sort_keys=False)
                f.write('\n')
        except Exception as exc:  # pragma: no cover
            try:
                sys.stderr.write('run_meta 写出失败: %r\n' % (exc,))
            except Exception:
                pass
        return path

    @staticmethod
    def read(path: str) -> Optional[Dict[str, Any]]:
        """读回 run_meta.json（自检用）；失败返回 None。"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception:
            return None
        return data if isinstance(data, dict) else None


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _ensure_dir(d: str) -> None:
    """确保目录存在。"""
    if d:
        os.makedirs(d, exist_ok=True)


def _numpy_version() -> str:
    """取已安装的 numpy 版本。"""
    import numpy as np
    return str(np.__version__)


def _jsonable(v: Any) -> Any:
    """把参数值转成 JSON 可序列化的类型（numpy 标量/数组特殊处理）。"""
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, (str, bool, int, float)) or v is None:
        return v
    # numpy 标量
    item = getattr(v, 'item', None)
    if callable(item):
        try:
            return _jsonable(v.item())
        except Exception:  # pragma: no cover
            pass
    tolist = getattr(v, 'tolist', None)
    if callable(tolist):
        try:
            return _jsonable(v.tolist())
        except Exception:  # pragma: no cover
            pass
    return str(v)
