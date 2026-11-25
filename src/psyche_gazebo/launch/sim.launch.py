"""Start Gazebo Fortress, spawn the hexapod, and bring up its controllers.

    ros2 launch psyche_gazebo sim.launch.py
    ros2 launch psyche_gazebo sim.launch.py rviz:=true

Once it's up, the 18 joints accept position commands on
/leg_position_controller/commands (std_msgs/Float64MultiArray, order in
psyche_description/config/controllers.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('psyche_description')
    gz_share = get_package_share_directory('psyche_gazebo')

    xacro_file = os.path.join(desc_share, 'urdf', 'hexapod.urdf.xacro')
    controllers_file = os.path.join(desc_share, 'config', 'controllers.yaml')
    rviz_config = os.path.join(desc_share, 'rviz', 'hexapod.rviz')
    default_world = os.path.join(gz_share, 'worlds', 'flat.sdf')

    world = LaunchConfiguration('world')
    spawn_z = LaunchConfiguration('spawn_z')
    rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_sim:=true',
                 ' controllers_file:=', controllers_file]),
        value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 3 --render-engine ogre ', world]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # Feet start ~3 cm above the ground; the robot settles onto them.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'psyche_hexapod', '-z', spawn_z],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    leg_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_position_controller'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('spawn_z', default_value='0.20'),
        DeclareLaunchArgument('rviz', default_value='false'),
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn,
        # Start controllers in order once the robot exists in Gazebo.
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster,
                                           on_exit=[leg_position_controller])),
        rviz_node,
    ])
