"""launch 入口共用的安装路径定位与参数解析。

这里只提供启动前的通用逻辑；节点编排与各入口的业务校验留在 launch 文件中。
路径函数不检查文件存在性，需校验功能包配置时使用 parameter_files.package_config。
"""

import os
from typing import Any

_PACKAGE_NAME = 'team43_bringup'


def share_dir(package: str = _PACKAGE_NAME) -> str:
    """从 ament 安装索引定位包的 share 目录。"""
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def default_config(name: str) -> str:
    """share/config/<name> 默认路径。"""
    return os.path.join(share_dir(), 'config', name)


def share_config(name: str) -> str:
    """把（可能是包内相对名的）配置文件解析成 share/config 下的绝对路径。"""
    return os.path.join(share_dir(), 'config', os.path.basename(str(name)))


def default_rviz() -> str:
    """share/rviz/gimbal_tracker.rviz 默认路径。"""
    return os.path.join(share_dir(), 'rviz', 'gimbal_tracker.rviz')


def as_bool(text: Any, default: bool = False) -> bool:
    """字符串转布尔。"""
    s = str(text).strip().lower() if text is not None else ''
    if s == '':
        return default
    if s in ('1', 'true', 'yes', 'on', 'y', 't'):
        return True
    if s in ('0', 'false', 'no', 'off', 'n', 'f'):
        return False
    raise ValueError(f'非法布尔参数: {text!r}')
