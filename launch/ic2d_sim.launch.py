import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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
        output='screen',
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

    # Run nodes
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])


