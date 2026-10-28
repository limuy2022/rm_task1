"""ament_python 安装声明（team43_judge_io）。"""

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'team43_judge_io'

setup(
    name=PACKAGE_NAME,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gimbal_team',
    maintainer_email='gimbal@example.com',
    description='赛题一输出契约：tick 驱动 9 列 CSV + 栅格地图落盘',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'judge_io_node = team43_judge_io.judge_io_node:main',
            'judge_sim = team43_judge_io.judge_sim:main',
        ],
    },
)
