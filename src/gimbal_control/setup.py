"""ament_python 安装声明：gimbal_control。"""

from setuptools import setup

PACKAGE_NAME = 'gimbal_control'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gimbal_tracker',
    maintainer_email='gimbal@example.com',
    description='云台控制：前馈+PD 控制律 + 限幅 + 搜索模式',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gimbal_controller_node = gimbal_control.gimbal_controller_node:main',
            'visual_servo_node = gimbal_control.visual_servo_node:main',
            'tracking_debug_node = gimbal_control.tracking_debug_node:main',
            'gimbal_bearing_node = gimbal_control.gimbal_bearing_node:main',
        ],
    },
)
