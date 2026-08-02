import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_behavior'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doriantodd',
    maintainer_email='doriantodd@todo.todo',
    description='Phase 6 behaviour layer: mode switching, PS controller bridge, dual ArUco tracking, and drive mux',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mode_manager_node = robot_behavior.mode_manager_node:main',
            'odin_target_tracker_node = robot_behavior.odin_target_tracker_node:main',
            'odin_yolo_tracker_node = robot_behavior.odin_yolo_tracker_node:main',
            'rear_yolo_tracker_node = robot_behavior.rear_yolo_tracker_node:main',
            'target_selector_node = robot_behavior.target_selector_node:main',
            'target_state_estimator_node = robot_behavior.target_state_estimator_node:main',
            'behavior_state_machine_node = robot_behavior.behavior_state_machine_node:main',
            'target_pid_node = robot_behavior.target_pid_node:main',
            'intent_to_goal_node = robot_behavior.intent_to_goal_node:main',
            'test_monitor_node = robot_behavior.test_monitor_node:main',
            'costmap_to_image_node = robot_behavior.costmap_to_image_node:main',
            'ps_controller_bridge_node = robot_behavior.ps_controller_bridge_node:main',
            'drive_mux_node = robot_behavior.drive_mux_node:main',
            'safety_watchdog_node = robot_behavior.safety_watchdog_node:main',
        ],
    },
)
