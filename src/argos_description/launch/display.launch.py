"""Launch robot_state_publisher with the ARGOS URDF.

Optionally opens rviz2 with a pre-configured view and
joint_state_publisher_gui for interactive joint manipulation.

Usage:
  ros2 launch argos_description display.launch.py
  ros2 launch argos_description display.launch.py gui:=false rviz:=false
  ros2 launch argos_description display.launch.py joint_states:=false
      (use when joint_state_estimator provides /joint_states externally)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, Command, LaunchConfiguration, \
    NotSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('argos_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'argos.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'argos.rviz')

    use_gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')
    use_joint_states = LaunchConfiguration('joint_states')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Launch joint_state_publisher_gui for interactive control'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch rviz2 with pre-configured view'),
        DeclareLaunchArgument(
            'joint_states', default_value='true',
            description='Launch a joint_state_publisher (set false when using '
                        'an external source like joint_state_estimator)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            }],
            output='screen',
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(AndSubstitution(use_joint_states, use_gui)),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=IfCondition(
                AndSubstitution(use_joint_states, NotSubstitution(use_gui))),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
