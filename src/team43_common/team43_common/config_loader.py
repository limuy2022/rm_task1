"""YAML 配置加载（PyYAML），用于 ``virtual.yaml`` / ``field.yaml`` / ``tracker_params.yaml``。

只依赖 PyYAML，不依赖 ROS，可在无 ROS 环境下单测。
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Sequence

import yaml

__all__ = ['ConfigError', 'load_yaml', 'deep_get', 'require_keys']


class ConfigError(RuntimeError):
    """配置文件缺失或结构非法（按 DESIGN §8.4：配置文件缺失应 FATAL 退出）。"""


def load_yaml(path: str, required: bool = True) -> Dict[str, Any]:
    """加载 YAML 文件。

    Args:
        path: 文件路径。
        required: True 时文件缺失抛 :class:`ConfigError`（FATAL）；False 时返回空 dict。

    Returns:
        dict: 解析结果（顶层非 dict 时返回空 dict）。

    Raises:
        ConfigError: 文件缺失（required=True）或内容不是映射。
    """
    if path is None or not os.path.isfile(path):
        if required:
            raise ConfigError(f'配置文件不存在: {path!r}')
        return {}
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigError(f'配置文件顶层必须是映射: {path!r}')
    return data


def deep_get(cfg: Dict[str, Any], key_path: str, default: Any = None,
             sep: str = '.') -> Any:
    """按点分路径取嵌套配置值，任一层缺失则返回 ``default``。

    Args:
        cfg: 配置字典。
        key_path: 如 ``'camera.fx'``。
        default: 缺失时的返回值。
        sep: 路径分隔符。

    Returns:
        Any: 取到的值或 default。
    """
    cur: Any = cfg
    for key in str(key_path).split(sep):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def require_keys(cfg: Dict[str, Any], keys: Sequence[str],
                 where: str = 'config', sep: str = '.') -> None:
    """校验必需的键都存在，缺失抛 :class:`ConfigError`。

    Args:
        cfg: 配置字典。
        keys: 必需的点分路径列表。
        where: 用于报错的配置名。
        sep: 路径分隔符。

    Raises:
        ConfigError: 任一必需键缺失。
    """
    missing: List[str] = []
    sentinel = object()
    for key in keys:
        if deep_get(cfg, key, sentinel, sep=sep) is sentinel:
            missing.append(key)
    if missing:
        raise ConfigError(f'{where} 缺少必需键: {missing}')
