#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():

    # --- Package paths ---
    sonar_pkg = get_package_share_directory('waterlinked_sonar_3d15')
    mavros_pkg = get_package_share_directory('sonar_localization')

    sonar = LaunchConfiguration('sonar')
    dvl = LaunchConfiguration('dvl')
    mavros = LaunchConfiguration('mavros')
    # --- Launch files ---
    sonar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sonar_pkg, 'launch', 'sonar_3d15.launch.py')
        ),
        condition=IfCondition(sonar)
    )

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavros_pkg, 'launch', 'mavros.launch.py')
        ),
        condition=IfCondition(mavros)
    )

    # --- Static transform ---
    base_link_to_sonar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_sonar',
        arguments=['0.165', '0.037', '-0.114', '0.0', '-0.3926991', '0.0', 'base_link', 'sonar_link'],
        output='screen',
    )

    base_link_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0.165', '-0.052', '-0.114', '1.5707963', '0.0', '1.1780972', 'base_link', 'hd_camera_link'],
        output='screen',
    )

    base_link_to_dvl = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_dvl',
        arguments=['0.0', '-0.106', '0.265', '0.0', '0.0', '0.0', 'base_link', 'dvl_link'],
        output='screen',
    )



    # --- Nucleus nodes ---
    nucleus_node = Node(
        package='nucleus_driver_ros2',
        executable='nucleus_node',
        name='nucleus_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(dvl),
    )

    connect_tcp = TimerAction(
        period=3.0,
        condition=IfCondition(dvl),
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
        condition=IfCondition(dvl),
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
        DeclareLaunchArgument(
            'dvl',
            default_value='true',
            description='connect to dvl',
        ),
        DeclareLaunchArgument(
            'sonar',
            default_value='true',
            description='connect to sonar',
        ),
        DeclareLaunchArgument(
            'mavros',
            default_value='true',
            description='connect to robot',
        ),
        sonar_launch,
        mavros_launch,
        base_link_to_sonar,
        base_link_to_dvl,
        base_link_to_camera,
        nucleus_node,
        connect_tcp,
        nucleus_start,       
    ])