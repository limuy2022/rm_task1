"""ament_python 安装声明（team43_dataio）。"""

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'team43_dataio'

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
    description='运行元数据与轨迹标注图',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_annotator_node = team43_dataio.trajectory_annotator_node:main',
        ],
    },
)
