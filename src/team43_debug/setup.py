"""ament_python 安装声明（team43_debug）。"""

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'team43_debug'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    description='角度误差指标与 GT 真值监测（唯一允许订阅真值话题的包）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gt_error_monitor_node = team43_debug.gt_error_monitor_node:main',
        ],
    },
)
