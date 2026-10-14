"""ament_python 安装声明（chassis_advisor）。"""

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'chassis_advisor'

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
    description='底盘建议速度（距离律 + 13 方向射线避障 + 平滑限幅）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_advisor_node = chassis_advisor.chassis_advisor_node:main',
        ],
    },
)
