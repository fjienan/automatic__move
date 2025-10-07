from setuptools import find_packages, setup

package_name = 'automove_engine'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/state_manager_launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'std_msgs'],
    zip_safe=True,
    maintainer='jienan',
    maintainer_email='fjienan@example.com',
    description='基于 ROS2 的自主移动机器人状态管理引擎',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quick_test = automove_engine.quick_test:main',
            'state_manager = automove_engine.state_manager:main',
            'state_test = automove_engine.state_test:main',
            'auto_move_engine = automove_engine.automove_engine:main',
        ],
    },
)
