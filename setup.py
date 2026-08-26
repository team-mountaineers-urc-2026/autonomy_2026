from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomy_2026'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='idw',
    maintainer_email='izaakwhetsell@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_p2p = autonomy_2026.static_p2p_vel:main',
            'local_path_planner = autonomy_2026.local_path_planner:main',
            'brian = autonomy_2026.brian_node:main',
            'pose_localizer = autonomy_2026.position_localizer:main',
            'pid_planner_node = autonomy_2026.new_pid_planner:main',
            'control_mux = autonomy_2026.move_controller_mux:main',
            'led_interface = autonomy_2026.led_interface:main',
            'velocity_mux = autonomy_2026.velocity_mux:main',
            'waypoint_manager = autonomy_2026.waypoint_manager:main',
            'camera_info = autonomy_2026.camera_info:main'
        ],
    },
)
