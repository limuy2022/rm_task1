"""ament_python 安装声明（grid_mapper）。"""

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'grid_mapper'

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
    zip_safe=True,
    maintainer='gimbal_team',
    maintainer_email='gimbal@example.com',
    description='场地栅格化与地图落盘（map.yaml / map.pgm / PNG）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_mapper_node = grid_mapper.grid_mapper_node:main',
        ],
    },
)
