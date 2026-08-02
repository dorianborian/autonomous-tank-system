# Phase 8 web control/telemetry UI. Standalone launch file so it can be
# added to either robot_behavior bringup file, or run on its own against an
# already-running stack:
#     ros2 launch robot_web_ui web_ui.launch.py
# Serves the page + WebSocket + MJPEG streams on http://<jetson-ip>:8088/
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    web_ui = Node(
        package='robot_web_ui',
        executable='web_ui_node',
        name='web_ui_bridge_node',
        output='screen',
    )
    return LaunchDescription([web_ui])
