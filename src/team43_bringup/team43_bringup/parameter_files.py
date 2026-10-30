"""通过 ament 安装索引定位各功能包拥有的配置文件。"""
import os
from ament_index_python.packages import get_package_share_directory


def package_config(package, filename='tracker_defaults.yaml'):
    """返回已安装配置的绝对路径，缺失时在启动阶段报错。"""
    path = os.path.join(get_package_share_directory(package), 'config', filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'{package} 配置未安装: {path}')
    return path
