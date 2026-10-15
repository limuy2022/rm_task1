"""pytest 路径注入：让 `pytest src/<pkg>/test` 在**未 colcon build** 的 Windows 环境下也能跑。

把本包的**父目录**（即 `src/<pkg>` 的上一级）加入 `sys.path`，从而
`import grid_mapper` / `import chassis_advisor` / `import gimbal_dataio` 可用。

⚠ 仅用于开发机单测；Ubuntu 上包已由 `colcon build --symlink-install` 安装，
本文件不产生任何副作用（路径重复时不会重复插入）。
"""

import os
import sys

_PKG_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)
