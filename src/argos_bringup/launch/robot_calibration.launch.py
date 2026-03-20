"""Robot-side nodes for calibration: hardware_bridge + sensors.

Run this on the robot (argos-ros.local) while running aruco_viz.launch.py
on the controller (atlas-controller).

Usage:
  ros2 launch argos_bringup robot_calibration.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='argos_hardware',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen',
        ),
        Node(
            package='argos_hardware',
            executable='flotilla_node',
            name='flotilla_node',
            parameters=[{'publish_rate': 50.0}],
            output='screen',
        ),
        Node(
            package='argos_hardware',
            executable='imu_node',
            name='imu_node',
            parameters=[{'publish_rate': 100.0}],
        ),
    ])
