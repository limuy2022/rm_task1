"""ament_python 安装声明（gimbal_dataio）。"""

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'gimbal_dataio'

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
    description='20Hz CSV 落盘（严格 8 列）、run_meta.json、轨迹标注图',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csv_writer_node = gimbal_dataio.csv_writer_node:main',
            'trajectory_annotator_node = gimbal_dataio.trajectory_annotator_node:main',
        ],
    },
)
