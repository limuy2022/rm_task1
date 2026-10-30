from glob import glob
from setuptools import setup

package_name = 'team43_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    description='视觉感知：ArUco 角点 -> solvePnP -> 各向异性观测噪声。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_pnp_node = team43_perception.marker_pnp_node:main',
            'aruco_detector_node = team43_perception.aruco_detector_node:main',
        ],
    },
)
