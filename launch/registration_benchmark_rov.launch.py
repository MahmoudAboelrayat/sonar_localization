import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('sonar_localization')
    ekf_config_path  = os.path.join(pkg_share, 'config', 'ekf_loop.yaml')
    bench_config_path = os.path.join(pkg_share, 'config', 'registration_benchmark_rov.yaml')

    rviz_flag  = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1', 'yes')
    sim_time   = LaunchConfiguration('sim_time').perform(context).lower() in ('true', '1', 'yes')
    dvl_type   = LaunchConfiguration('dvl_type').perform(context).lower()
    method     = LaunchConfiguration('method').perform(context).upper()  # VGICP | GICP | ICP | NDT | ICPNL

    return [

        #### Topic Bridges ####
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensors_bridge.launch.py')
            ),
            launch_arguments={
                'dvl_type':       dvl_type,
                'dvl_frame':      'dvl_link',
                'depth_frame':    'icp_map',
                'relative_depth': 'false',
                'depth_topic':    '/global_position/rel_alt',
                'ned':            'false',
            }.items(),
        ),

        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': sim_time}],
            remappings=[('odometry/filtered', 'odometry/ekf_local')]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': sim_time}]
        ),

        #### TF ####
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Beckholmen_tf',
            arguments=['0', '0', '0', '0.0', '0', '3.14159265359', 'world', 'Beckholmen'],
            parameters=[{'use_sim_time': sim_time}]
        ),
        Node(
            package='sonar_localization',
            executable='odom_tf',
            name='map_tf',
            output='screen',
            parameters=[{
                'parent_frame': 'world',
                'child_frame':  'icp_map',
                'use_sim_time': sim_time,
                'init_x': 0.0, 'init_y': 0.0, 'init_z': 0.0,
                'init_yaw': 0.0, 'init_roll': 0.0, 'init_pitch': 0.0,
            }]
        ),

        #### Registration Benchmark ####
        Node(
            package='sonar_localization',
            executable='registration_benchmark',
            name='vgicp_odom_node',
            output='screen',
            parameters=[bench_config_path, {'registration.method': method}]
        ),

        #### RViz ####
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'rov_dry_dock.rviz')],
            parameters=[{'use_sim_time': sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dvl_type',  default_value='waterlinked',
                              description='DVL model: nucleus | waterlinked | sim'),
        DeclareLaunchArgument('method',    default_value='VGICP',
                              description='Registration method: VGICP | GICP | ICP | NDT | ICPNL'),
        DeclareLaunchArgument('rviz',      default_value='true',
                              description='Open RViz window'),
        DeclareLaunchArgument('sim_time',  default_value='false',
                              description='Use sim_time'),
        OpaqueFunction(function=launch_setup),
    ])
