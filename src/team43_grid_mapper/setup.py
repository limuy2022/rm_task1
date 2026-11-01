"""ament_python 安装声明（team43_grid_mapper）。"""

from setuptools import find_packages
from glob import glob
from setuptools import setup

PACKAGE_NAME = 'team43_grid_mapper'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    description='场地栅格化与地图落盘（map.yaml / map.pgm / PNG）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_mapper_node = team43_grid_mapper.grid_mapper_node:main',
            'cloud_mapping_node = team43_grid_mapper.cloud_mapping_node:main',
            'saved_map_node = team43_grid_mapper.saved_map_node:main',
        ],
    },
)
