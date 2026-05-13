import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('sonar_localization')

    sim_time = LaunchConfiguration('sim_time').perform(context).lower() in ('true', '1', 'yes')
    dvl_type = LaunchConfiguration('dvl').perform(context).lower()  # 'nucleus' or 'waterlinked' or 'sim'

    dvl_topic = LaunchConfiguration('dvl_topic')
    dvl_frame = LaunchConfiguration('dvl_frame')

    depth_topic = LaunchConfiguration('depth_topic')
    depth_frame = LaunchConfiguration('dvl_frame')
    relative_depth = LaunchConfiguration('relative_depth').perform(context).lower() in ('true', '1', 'yes')
    ned = LaunchConfiguration('relative_depth').perform(context).lower() in ('true', '1', 'yes')


    if dvl_type == 'nucleus':
        dvl_ex = 'nucleus_dvl_bridge'
    elif dvl_type == 'waterlinked':
        dvl_ex = 'waterlinked_dvl_bridge'
    else:
        dvl_ex = 'sim_dvl_briddg'

    return [

        Node(
            package='sonar_localization',
            executable=dvl_ex,
            name='dvl_bridge',
            output='screen',
            parameters=[{'frame_id':dvl_frame, 'input_topic': dvl_topic}],
        ),

        Node(
            package='sonar_localization',
            executable='depth_bridge',
            name='depth_bridge',
            output='screen',
            parameters=[{'frame_id': 'icp_map','rel_alt_topic': '/mavros/global_position/rel_alt','relative_depth':True,"ned": full_bag}],),

    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dvl_type', default_value='waterlinked',
                              description='dvl model: (nucleus) or (waterlinked) or (sim)'),

        DeclareLaunchArgument('dvl_topic', default_value='/saabmarine/core/dvl',
                              description='dvl_topic'),

        DeclareLaunchArgument('dvl_frame', default_value='saabmarine/dvl_frame',
                                    description='dvl TF frame'),

        DeclareLaunchArgument('depth_topic', default_value='/saabmarine/core/dvl',
                              description='depth senosr topic'),

        DeclareLaunchArgument('dvl_frame', default_value='saabmarine/dvl_frame',
                                    description='depth TF frame'),

        DeclareLaunchArgument('relative_depth', default_value='True',
                                    description='is the depth relative or absolute'),
        DeclareLaunchArgument('ned', default_value='True',
                                    description='is depth reading ned frame'),



        OpaqueFunction(function=launch_setup),
    ])





