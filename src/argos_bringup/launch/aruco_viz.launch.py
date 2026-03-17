"""Launch ArUco marker visualization: URDF + camera + aruco_node + rviz2.

Use this to verify ArUco marker placement and orientation on the arm
before feeding into joint pose estimation.

Usage:
  ros2 launch argos_bringup aruco_viz.launch.py
  ros2 launch argos_bringup aruco_viz.launch.py marker_size:=0.02 gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('argos_description')
    xacro_file = os.path.join(desc_share, 'urdf', 'argos.urdf.xacro')
    rviz_config = os.path.join(desc_share, 'rviz', 'argos_aruco.rviz')

    marker_size = LaunchConfiguration('marker_size')
    use_gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'marker_size', default_value='0.02',
            description='ArUco marker side length in metres'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Launch joint_state_publisher_gui to manually pose the URDF arm'),

        # URDF → /robot_description + /tf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            }],
            output='screen',
        ),

        # Joint GUI so you can manually pose the URDF arm for comparison
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),

        # Camera
        Node(
            package='argos_hardware',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'device_index': 0,
                'width': 640,
                'height': 480,
                'fps': 30,
                'publish_rate': 15.0,
            }],
        ),

        # ArUco detection
        Node(
            package='argos_hardware',
            executable='aruco_node',
            name='aruco_node',
            parameters=[{
                'marker_size': marker_size,
                'dictionary': 'DICT_4X4_50',
                'marker_ids': [0, 1, 2, 3],
                'joint_names': ['shoulder', 'elbow', 'wrist', 'gripper'],
            }],
            output='screen',
        ),

        # rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
