import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    ExecuteProcess, 
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Load Xacro file:
    xacro_file = os.path.join(
        get_package_share_directory('ic2d_urdf'), 
        'description/ic2d.xacro'
    )
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch'),
                '/gazebo.launch.py']),
        )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'ic2d']
    )

    rqt_joint_controller = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        output='screen',
    )

    joint_state_controller = ExecuteProcess(
        cmd=[
            'ros2',
            'control',
            'load_controller',
            '--set-state',
            'active',
            'joint_state_broadcaster',
        ],
        output='both',
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=[
            'ros2',
            'control',
            'load_controller',
            '--set-state',
            'active',
            'joint_trajectory_controller',
        ],
        output='both',
    )

    # Run nodes
    return LaunchDescription([
        gazebo,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_controller],
                )
            ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_controller,
                on_exit=[joint_trajectory_controller],
            )
        ),
        robot_state_publisher,
        rqt_joint_controller,
        spawn_entity,
    ])


