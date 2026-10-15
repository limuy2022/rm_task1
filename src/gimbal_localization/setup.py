from setuptools import setup

package_name = 'gimbal_localization'

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
    description='底盘位姿源（odom/mock）+ map 系 t0 冻结与广播。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_pose_provider_node = gimbal_localization.base_pose_provider_node:main',
            'map_frame_broadcaster_node = gimbal_localization.map_frame_broadcaster_node:main',
        ],
    },
)
