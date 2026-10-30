"""ament_python 安装声明：team43_estimation。"""

from glob import glob
from setuptools import setup

PACKAGE_NAME = 'team43_estimation'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    description='目标状态估计：观测换算 + CA-KF + 100ms 外推',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_estimator_node = team43_estimation.target_estimator_node:main',
        ],
    },
)
