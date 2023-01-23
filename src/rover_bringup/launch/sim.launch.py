from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rover_description"), "urdf", "rover.urdf.xacro"]
            ),
            " ",
            "is_sim:=true"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        # respawn=True,
        output='screen',
    )

    spawn_rover_velocity_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['rover_velocity_controller', '-c', '/controller_manager'],
        # respawn=True,
        output='screen',
    )

    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_rover_velocity_controller],
        )
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
                )
             )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity',
                   'rover',
                   '-topic',
                   'robot_description'],
        output='screen',
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            gazebo,
            spawn_robot,
            spawn_joint_state_broadcaster,
            spawn_rover_velocity_controller,
            diffdrive_controller_spawn_callback
        ]
    )