import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('sonar_localization')
    ekf_config_path   = os.path.join(pkg_share, 'config', 'ekf_loop.yaml')
    bench_config_path = os.path.join(pkg_share, 'config', 'registration_benchmark_rov.yaml')

    full_bag = LaunchConfiguration('full_bag').perform(context).lower() in ('true', '1', 'yes')
    method   = LaunchConfiguration('method').perform(context).upper()

    if full_bag:
        init_x, init_y, init_z, init_yaw = 52.670, -2.4, 0.0, -0.096
    else:
        init_x, init_y, init_z, init_yaw = 38.529, -2.881, -1.5, -3.031

    return [

        #### Topic Bridges ####
        Node(
            package='sonar_localization',
            executable='dvl_bridge',
            name='dvl_bridge',
            output='screen',
            parameters=[{'sim': False, 'frame_id': 'saabmarine/dvl_frame'}],
        ),
        Node(
            package='sonar_localization',
            executable='depth_bridge',
            name='depth_bridge',
            output='screen',
            parameters=[{
                'frame_id':       'icp_map',
                'rel_alt_topic':  '/mavros/global_position/rel_alt',
                'relative_depth': True,
                'ned':            full_bag,
            }],
        ),

        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}],
            remappings=[('odometry/filtered', 'odometry/ekf_local')]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}]
        ),

        #### Static TF ####
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0', '0.0', '0', '3.14159265359',
                       'saabmarine/base_link', 'base_link'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments=['0', '0', '0', '3.1415', '0', '0.0',
                       'saabmarine/base_link', 'gps'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Beckholmen_tf',
            arguments=['0', '0', '0', '0.0', '0', '3.14159265359', 'world', 'Beckholmen'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar',
            arguments=['0.220', '0', '-0.160', '0.0', '-0.524', '0.0',
                       'saabmarine/base_link', 'saabmarine/sonar_link'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_dvl',
            arguments=['-0.150', '-0.150', '0.150', '0.0', '0.0', '0.0',
                       'saabmarine/base_link', 'saabmarine/dvl_frame'],
            parameters=[{'use_sim_time': True}]
        ),

        #### Map origin TF ####
        Node(
            package='sonar_localization',
            executable='odom_tf',
            name='map_tf',
            output='screen',
            parameters=[{
                'parent_frame': 'Beckholmen',
                'child_frame':  'icp_map',
                'use_sim_time': True,
                'init_x':       init_x,
                'init_y':       init_y,
                'init_z':       init_z,
                'init_yaw':     init_yaw,
                'init_roll':    0.0,
                'init_pitch':   0.0,
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
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='dry_dock_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(
                    os.path.join(pkg_share, 'description', 'dry_dock.urdf')
                ).read(),
                'frame_id': 'Beckholmen',
            }]
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('full_bag', default_value='true',
                              description='true = full bag (init at 52.67,-2.4), false = cropped bag'),
        DeclareLaunchArgument('method',   default_value='VGICP',
                              description='Registration method: VGICP | GICP | ICP | NDT | ICPNL'),
        OpaqueFunction(function=launch_setup),
    ])
