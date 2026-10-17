from setuptools import setup

package_name = 'gimbal_common'

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
    description='跨包共享层：协议、时钟、数学、帧常量、配置、日志、CSV。',
    license='Apache-2.0',
    tests_require=['pytest'],
)
