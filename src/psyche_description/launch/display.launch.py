"""View the hexapod in RViz with a slider for every joint. No Gazebo, no physics.

    ros2 launch psyche_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('psyche_description')
    xacro_file = os.path.join(share, 'urdf', 'hexapod.urdf.xacro')
    rviz_config = os.path.join(share, 'rviz', 'hexapod.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
