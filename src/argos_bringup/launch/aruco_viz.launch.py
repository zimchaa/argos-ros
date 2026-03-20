"""Launch ArUco marker visualization: URDF + camera + aruco + joint estimator + rviz2.

Launches the full ArUco-based arm tracking pipeline with joint state estimation.
Use gui:=true to also launch the joint_state_publisher_gui for manual comparison.
Use camera:=false to skip the camera node (when camera runs on the robot).

Usage:
  ros2 launch argos_bringup aruco_viz.launch.py
  ros2 launch argos_bringup aruco_viz.launch.py marker_size:=0.02 gui:=true
  ros2 launch argos_bringup aruco_viz.launch.py camera:=false   # camera on robot
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
    hw_share = get_package_share_directory('argos_hardware')
    xacro_file = os.path.join(desc_share, 'urdf', 'argos.urdf.xacro')
    rviz_config = os.path.join(desc_share, 'rviz', 'argos_aruco.rviz')
    estimator_config = os.path.join(hw_share, 'config', 'joint_state_estimator.yaml')

    marker_size = LaunchConfiguration('marker_size')
    use_gui = LaunchConfiguration('gui')
    use_camera = LaunchConfiguration('camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'marker_size', default_value='0.02',
            description='ArUco marker side length in metres'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Also launch joint_state_publisher_gui for manual comparison'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Launch camera node (set false when camera runs on robot)'),

        # URDF → /robot_description + /tf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            }],
            output='screen',
        ),

        # Joint state estimator — ArUco TF → /joint_states (with saved calibration)
        Node(
            package='argos_hardware',
            executable='joint_state_estimator',
            name='joint_state_estimator',
            parameters=[estimator_config],
            output='screen',
        ),

        # Joint GUI for manual comparison (off by default, conflicts with estimator)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),

        # Camera (skip when running on a different machine from the robot)
        Node(
            package='argos_hardware',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'device_index': 0,
                'width': 640,
                'height': 480,
                'fps': 30,
                'publish_rate': 5.0,
            }],
            condition=IfCondition(use_camera),
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
