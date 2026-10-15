"""ament_python 安装声明：gimbal_estimation。"""

from setuptools import setup

PACKAGE_NAME = 'gimbal_estimation'

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
    description='目标状态估计：观测换算 + CA-KF + 100ms 外推',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_estimator_node = gimbal_estimation.target_estimator_node:main',
        ],
    },
)
