"""Launch all sensor nodes: IMU, Flotilla, AHRS, sonar, IR, camera."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='argos_hardware',
            executable='imu_node',
            name='imu_node',
            parameters=[{'publish_rate': 100.0}],
        ),
        Node(
            package='argos_hardware',
            executable='flotilla_node',
            name='flotilla_node',
            parameters=[{'publish_rate': 50.0}],
        ),
        Node(
            package='argos_hardware',
            executable='ahrs_node',
            name='ahrs_node',
            parameters=[{'publish_rate': 50.0, 'beta': 0.05}],
        ),
        Node(
            package='argos_hardware',
            executable='sonar_node',
            name='sonar_node',
            parameters=[{'publish_rate': 10.0}],
        ),
        Node(
            package='argos_hardware',
            executable='ir_node',
            name='ir_node',
            parameters=[{'publish_rate': 20.0}],
        ),
        Node(
            package='argos_hardware',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'device_index': 0,
                'width': 640,
                'height': 480,
                'fps': 30,
                'publish_rate': 30.0,
            }],
        ),
    ])
