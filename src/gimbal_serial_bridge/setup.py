from setuptools import setup

package_name = 'gimbal_serial_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gimbal_tracker_team',
    maintainer_email='team@example.com',
    description='虚拟云台串口桥：收包解码 + 100Hz 指令下发 + 链路状态机。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vgimbal_bridge_node = gimbal_serial_bridge.vgimbal_bridge_node:main',
        ],
    },
)
