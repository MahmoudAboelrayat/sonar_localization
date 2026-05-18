#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # --- Package paths ---
    sonar_pkg = get_package_share_directory('waterlinked_sonar_3d15')
    mavros_pkg = get_package_share_directory('sonar_localization')

    # --- Launch files ---
    sonar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sonar_pkg, 'launch', 'sonar_3d15.launch.py')
        )
    )

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavros_pkg, 'launch', 'mavros.launch.py')
        )
    )

    # --- Static transform ---
    base_link_to_sonar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_sonar',
        arguments=['0.220', '0', '-0.160', '0.0', '-0.524', '0.0', 'base_link', 'sonar_link'],
        output='screen',
    )

    # --- Nucleus nodes ---
    nucleus_node = Node(
        package='nucleus_driver_ros2',
        executable='nucleus_node',
        name='nucleus_node',
        output='screen',
        emulate_tty=True,
    )

    connect_tcp = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nucleus_driver_ros2',
                executable='connect_tcp',
                name='nucleus_connect',
                arguments=['192.168.32.23', 'nortek'],
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    nucleus_start = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='nucleus_driver_ros2',
                executable='start',
                name='nucleus_start',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        sonar_launch,
        mavros_launch,
        base_link_to_sonar,
        nucleus_node,
        connect_tcp,
        nucleus_start,
    ])