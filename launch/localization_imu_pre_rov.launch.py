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
    imu_config_path  = os.path.join(pkg_share, 'config', 'localization_imu_pre_rov.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')

    rviz_flag  = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1', 'yes')
    use_dock   = LaunchConfiguration('dock').perform(context).lower() in ('true', '1', 'yes')
    full_bag   = LaunchConfiguration('full_bag').perform(context).lower() in ('true', '1', 'yes')
    sim_time   = LaunchConfiguration('sim_time').perform(context).lower() in ('true', '1', 'yes')
    dvl_type   = LaunchConfiguration('dvl_type').perform(context).lower()

    if full_bag:
        init_x, init_y, init_z, init_yaw = 52.670, -2.4, 0.0, -0.096
    else:
        init_x, init_y, init_z, init_yaw = 0.0, 0.0, 0.0, 0.0

    return [

        #### Topics Bridges ####
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_share, 'launch', 'sensors_bridge.launch.py')
        #     ),
        #     launch_arguments={
        #         'dvl_type':       dvl_type,
        #         'dvl_frame':      'dvl_link',
        #         'depth_frame':    'icp_map',
        #         'relative_depth': 'false',
        #         'depth_topic':    '/global_position/rel_alt',
        #         'ned':            'false',
        #     }.items(),
        # ),

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
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_dvl',
        #     arguments=['-0.150', '-0.150', '0.150', '0.0', '0.0', '0.0',
        #                'base_link', dvl_link'],
        #     parameters=[{'use_sim_time': sim_time}]
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_dvl_imu',
        #     arguments=['-0.150', '-0.150', '0.150', '0.0', '0.0', '0.0',
        #                'base_link', 'dvl_link'],
        #     parameters=[{'use_sim_time': sim_time}]
        # ),

         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar',
            arguments=['0.165', '0.037', '-0.114', '0.0', '-0.3926991', '0.0', 'base_link', 'sonar_link'],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=['0.165', '-0.052', '-0.114', '1.5707963', '0.0', '1.1780972', 'base_link', 'hd_camera_link'],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_dvl',
            arguments=['0.0', '-0.106', '0.265', '0.0', '0.0', '0.0', 'base_link', 'dvl_link'],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gss',
            arguments=['0.0', '-0.106', '-0.2', '0.0', '0.0', '3.1415', 'base_link', 'gnss'],
            output='screen',
        ),

        #### Map TF ####
        Node(
            package='sonar_localization',
            executable='odom_tf',
            name='map_tf',
            output='screen',
            parameters=[{
                'parent_frame': 'world',
                'child_frame':  'icp_map',
                'use_sim_time': sim_time,
                'init_x':       init_x,
                'init_y':       init_y,
                'init_z':       init_z,
                'init_yaw':     init_yaw,
                'init_roll':    0.0,
                'init_pitch':   0.0,
            }]
        ),

        #### SLAM node with IMU preintegration + DVL ####
        Node(
            package='sonar_localization',
            executable='localization_imu_pre',
            name='localization_imu_pre',
            output='screen',
            parameters=[imu_config_path, {'use_sim_time': sim_time}]
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

        #### Dry dock model ####
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
            }],
            condition=IfCondition(LaunchConfiguration('dock'))
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': sim_time}],
            remappings=[
                ('imu',           '/dvl/imu'),
                ('gps/fix',            'fix'),
                ('odometry/filtered',  'odometry/filtered'),
            ],
        ),

        Node(
            package='sonar_localization',
            executable='odom_enu2ned',
            name='odom_enu2ned',
            output='screen',
            parameters=[{
                'input_topic':  '/odometry/gps',
                'output_topic': '/odometry/gps_ned',
                'output_frame': 'icp_map',
                'use_sim_time': sim_time,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dvl_type',  default_value='nucleus',
                              description='dvl model: nucleus / waterlinked / sim'),
        DeclareLaunchArgument('full_bag',  default_value='false',
                              description='true = full bag, false = cropped bag'),
        DeclareLaunchArgument('rviz',      default_value='true',
                              description='open rviz window'),
        DeclareLaunchArgument('sim_time',  default_value='true',
                              description='use sim_time'),
        DeclareLaunchArgument('dock',      default_value='false',
                              description='show dry dock model'),
        OpaqueFunction(function=launch_setup),
    ])
