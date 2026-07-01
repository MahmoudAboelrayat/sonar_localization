import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read_median_filter_config(config_path):
    """Read the median filter toggle and topic names from the param YAML.

    Returns (enabled, input_topic, output_topic) with sensible fallbacks so the
    launch still works if the section is missing.
    """
    enabled = True
    input_topic = '/usv/point_cloud'
    output_topic = '/usv/point_cloud_median'
    try:
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        params = data.get('angular_range_median_filter', {}).get('ros__parameters', {})
        enabled = bool(params.get('enabled', enabled))
        input_topic = params.get('input_topic', input_topic)
        output_topic = params.get('output_topic', output_topic)
    except (OSError, yaml.YAMLError):
        pass
    return enabled, input_topic, output_topic


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config').perform(context)
    enabled, input_topic, output_topic = _read_median_filter_config(config_path)

    # When the median filter is on, the validator consumes its output;
    # when off, the validator consumes the raw sonar cloud directly.
    validator_input_topic = output_topic if enabled else input_topic

    nodes = [
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
                'topics.pc_sub': validator_input_topic,
            }],
        ),
    ]

    if enabled:
        nodes.append(Node(
            package='dufomap_ros',
            executable='angular_range_median_node',
            name='angular_range_median_filter',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'use_sim_time': LaunchConfiguration('sim_time'),
            }],
        ))

    return nodes


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
        OpaqueFunction(function=_launch_setup),
    ])
