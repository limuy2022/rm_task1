# -*- coding: utf-8 -*-
"""pytest 引导：把 ``src/gimbal_estimation`` 与 ``src`` 加入 ``sys.path``。

``gimbal_common`` 是同工作区的纯 Python 包（仅依赖 numpy，rclpy 为惰性导入），
真实包总能被找到，无需桩兜底。
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))          # .../src/gimbal_estimation/test
_PKG_ROOT = os.path.dirname(_HERE)                           # .../src/gimbal_estimation
_SRC_ROOT = os.path.dirname(_PKG_ROOT)                       # .../src

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
