"""Launch the hardware bridge node (motor control + emergency stop)."""

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
    ])
