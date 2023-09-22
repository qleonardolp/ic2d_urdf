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
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'), 
                'launch',
                'gazebo.launch.py',
            ),
        ),
        launch_arguments={'world': 
            os.path.join(
                get_package_share_directory('ic2d_urdf'), 
                'world', 'ic2d_simulation.world'
            )
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'ic2d']
    )

    ic2d_effort_controller = Node(
        package='ic2d_urdf',
        executable='ic2d_effort_controller',
        output='screen',
    )

    joint_state_broadcaster = ExecuteProcess(
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
    
    ros2_control_effort = ExecuteProcess(
        cmd=[
            'ros2', 
            'control', 
            'load_controller', 
            '--set-state', 
            'active', 
            'effort_controller'
        ],
        output='both'
    )

    # Run nodes
    return LaunchDescription([
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster],
                )
            ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[ros2_control_effort],
            )
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])