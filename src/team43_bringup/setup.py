"""ament_python 安装声明（team43_bringup）。

除 Python 包本身，本包还把 `config/`、`launch/`、`rviz/` 安装到 `share/<pkg>/` 下，
使 `ros2 launch`、`get_package_share_directory()` 与 RViz `-d` 都能按标准路径取到文件
（DESIGN §3.13 / §8.6 单一数据源）。
"""

import os
from glob import glob

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'team43_bringup'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'config'), glob('config/*.yaml')),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', PACKAGE_NAME, 'launch', 'include'),
         glob('launch/include/*.launch.py')),
        (os.path.join('share', PACKAGE_NAME, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    description='一键启动与整车共享配置（launch / YAML / RViz）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'configure_livox = team43_bringup.livox_config:main',
        ],
    },
)
