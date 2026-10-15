# -*- coding: utf-8 -*-
"""pytest 引导：**不依赖 rclpy** 直接跑通 ``test_control_law.py``。

做的事情只有两件：
1. 把 ``src/gimbal_control`` 与 ``src`` 加入 ``sys.path``，使
   ``import gimbal_control`` / ``import gimbal_common`` 可用；
2. 若环境中**不存在**真实的 ``gimbal_common``（Windows 无 ROS2），
   把 ``test/_stub/gimbal_common`` 挂到 ``sys.path`` 最前面作为兜底。

在 Ubuntu + ROS2 工作区里，真实 ``gimbal_common`` 会被优先找到，本桩自动让位。
"""

import importlib.util
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))          # .../src/gimbal_control/test
_PKG_ROOT = os.path.dirname(_HERE)                           # .../src/gimbal_control
_SRC_ROOT = os.path.dirname(_PKG_ROOT)                       # .../src
_STUB_ROOT = os.path.join(_HERE, '_stub')

# ament_python 的 Python 包在 src/<pkg>/<pkg>/ 下，两个层级都要能找到
_PATHS = [_PKG_ROOT, _SRC_ROOT]
if os.path.isdir(_SRC_ROOT):
    for _d in sorted(os.listdir(_SRC_ROOT)):
        _inner = os.path.join(_SRC_ROOT, _d, _d)
        if os.path.isdir(_inner):
            _PATHS.append(os.path.join(_SRC_ROOT, _d))

for _p in _PATHS:
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _module_available(name: str) -> bool:
    """判断模块是否可被导入（不真正执行）。"""
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


if not _module_available('gimbal_common'):
    if _STUB_ROOT not in sys.path:
        sys.path.insert(0, _STUB_ROOT)
