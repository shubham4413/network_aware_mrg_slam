from setuptools import find_packages, setup

package_name = 'distance_bandwidth_shaper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shubham Nagar',
    maintainer_email='shubhamnagar@todo.todo',
    description='Distance-based bandwidth shaping and GUI for multi-robot graph SLAM',
    license='BSD-2-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'distance_bandwidth_shaper = distance_bandwidth_shaper.distance_bandwidth_shaper_node:main',
            'mrg_slam_gui = distance_bandwidth_shaper.mrg_slam_gui:main',
            'robot_planned_path = distance_bandwidth_shaper.robot_planned_path_updated:main',
        ],
    },
)
