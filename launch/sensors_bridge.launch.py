import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    dvl_type = LaunchConfiguration('dvl_type').perform(context).lower()  # 'nucleus' or 'waterlinked' or 'sim'

    dvl_topic = LaunchConfiguration('dvl_topic').perform(context)
    dvl_frame = LaunchConfiguration('dvl_frame').perform(context)

    depth_topic = LaunchConfiguration('depth_topic').perform(context)
    depth_frame = LaunchConfiguration('depth_frame').perform(context)
    relative_depth = LaunchConfiguration('relative_depth').perform(context).lower() in ('true', '1', 'yes')
    ned = LaunchConfiguration('ned').perform(context).lower() in ('true', '1', 'yes')

    if dvl_type == 'nucleus':
        dvl_ex = 'nucleus_dvl_bridge'
    elif dvl_type == 'waterlinked':
        dvl_ex = 'waterlinked_dvl_bridge'
    else:
        dvl_ex = 'sim_dvl_bridge'

    dvl_params = {'relative_depth': relative_depth, 'ned': ned}
    if dvl_topic:
        dvl_params['input_topic'] = dvl_topic
    if dvl_frame:
        dvl_params['frame_id'] = dvl_frame

    depth_params = {'relative_depth': relative_depth, 'ned': ned}
    if depth_topic:
        depth_params['input_topic'] = depth_topic
    if depth_frame:
        depth_params['frame_id'] = depth_frame

    return [

        Node(
            package='sonar_localization',
            executable=dvl_ex,
            name='dvl_bridge',
            output='screen',
            parameters=[dvl_params],
        ),

        Node(
            package='sonar_localization',
            executable='depth_bridge',
            name='depth_bridge',
            output='screen',
            parameters=[depth_params],
        ),

    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dvl_type', default_value='waterlinked',
                              description='dvl model: (nucleus) or (waterlinked) or (sim)'),

        DeclareLaunchArgument('dvl_topic', default_value='',
                              description='dvl input topic (leave empty to use node default)'),

        DeclareLaunchArgument('dvl_frame', default_value='',
                              description='dvl TF frame (leave empty to use node default)'),

        DeclareLaunchArgument('depth_topic', default_value='',
                              description='depth sensor topic (leave empty to use node default)'),

        DeclareLaunchArgument('depth_frame', default_value='',
                              description='depth TF frame (leave empty to use node default)'),

        DeclareLaunchArgument('relative_depth', default_value='True',
                                    description='is the depth relative or absolute'),
        DeclareLaunchArgument('ned', default_value='True',
                                    description='is depth reading ned frame'),



        OpaqueFunction(function=launch_setup),
    ])





