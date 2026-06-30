import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sonar_localization')
    config_path = os.path.join(pkg_share, 'config', 'dufomap_validator.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=config_path,
        description='Path to dufomap validator parameter file')

    sim_time_arg = DeclareLaunchArgument(
        'sim_time',
        default_value='true',
        description='Use simulation time for bag playback')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/imu/odometry',
        description='Odometry topic to broadcast as dynamic TF')

    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='false',
        description='Broadcast odom as TF (disable when bag provides /tf)')

    publish_sonar_tf_arg = DeclareLaunchArgument(
        'publish_sonar_tf',
        default_value='false',
        description='Publish static base_link -> sonar TF (disable when bag provides /tf_static)')

    return LaunchDescription([
        config_arg,
        sim_time_arg,
        odom_topic_arg,
        publish_odom_tf_arg,
        publish_sonar_tf_arg,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar',
            arguments=[
                '0.38', '0.08', '-0.525',
                '0.0', '0.52', '3.14159',
                'base_link', 'usv/sonar_link',
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}],
            condition=IfCondition(LaunchConfiguration('publish_sonar_tf')),
        ),
        Node(
            package='sonar_localization',
            executable='odom_tf_broadcaster.py',
            name='odom_tf_broadcaster',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_odom_tf')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('sim_time'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            }],
        ),
        Node(
            package='sonar_localization',
            executable='dufomap_submap_validator',
            name='dufomap_submap_validator',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'use_sim_time': LaunchConfiguration('sim_time'),
            }],
        ),
    ])
