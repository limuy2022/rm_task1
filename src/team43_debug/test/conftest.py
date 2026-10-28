"""pytest 路径注入：让 `pytest src/team43_debug/test` 在**未 colcon build** 的环境也能跑。

把 `src/team43_debug`（本包根）与 `src/team43_common`（依赖）加入 `sys.path`，
从而 `import team43_debug.metrics` 与 `import team43_common.frames` 均可用。

⚠ 仅用于开发机单测；Ubuntu 上包已由 `colcon build --symlink-install` 安装，
本文件不产生副作用（路径重复时不会重复插入）。
"""

import os
import sys

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.abspath(os.path.join(_TEST_DIR, '..'))
_SRC_DIR = os.path.abspath(os.path.join(_PKG_DIR, '..'))

for _p in (_PKG_DIR, os.path.join(_SRC_DIR, 'team43_common')):
    if _p not in sys.path:
        sys.path.insert(0, _p)
