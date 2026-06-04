import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    pkg_share     = get_package_share_directory('sonar_localization')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')
    ekf_config    = os.path.join(pkg_share, 'config', 'ekf_usv.yaml')
    rviz_config   = os.path.join(pkg_share, 'rviz',   'usv_mapping.rviz')

    sim_time = LaunchConfiguration('sim_time').perform(context).lower() in ('true', '1', 'yes')

    return [

        # ── Mapping node ────────────────────────────────────────────────────────
        Node(
            package='sonar_localization',
            executable='odom_mapping',
            name='odom_mapping_node',
            output='screen',
            parameters=[{
                'use_sim_time':   sim_time,
                'pc_topic':       LaunchConfiguration('pc_topic'),
                'odom_topic':     'odometry/filtered',  # from EKF
                'map_topic':      '/prior_map',
                'odom_frame':     LaunchConfiguration('odom_frame'),
                'map_resolution': LaunchConfiguration('map_resolution'),
                'publish_every':  5,
                # sonar → base_link extrinsic
                'sonar2base_tx':  -0.056,
                'sonar2base_ty':   0.000,
                'sonar2base_tz':  -0.311,
                'sonar2base_qw':   0.966,
                'sonar2base_qx':  -0.000,
                'sonar2base_qy':   0.259,
                'sonar2base_qz':  -0.000,
            }],
        ),

        # ── EKF node ────────────────────────────────────────────────────────────
        # Fuses IMU + DVL → /odometry/filtered  (fed into navsat as heading ref)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': sim_time}],
        ),

        # ── navsat_transform_node ───────────────────────────────────────────────
        # Converts GPS fixes into odometry in the odom frame.
        # Requires: /gps/fix  +  /imu/data  +  /odometry/filtered
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_config, {'use_sim_time': sim_time}],
            remappings=[
                ('imu',           LaunchConfiguration('imu_topic')),
                ('gps/fix',            LaunchConfiguration('gps_topic')),
                ('odometry/filtered',  LaunchConfiguration('odom_topic')),
            ],
        ),

        # ── RViz ───────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim_time',       default_value='true',
                              description='Use /clock (true for bag playback)'),
        DeclareLaunchArgument('rviz',           default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('pc_topic',       default_value='usv/point_cloud',
                              description='Incoming point cloud topic (sonar frame)'),
        DeclareLaunchArgument('odom_topic',     default_value='/imu/odometry',
                              description='Odometry topic (base_link in odom frame)'),
        DeclareLaunchArgument('imu_topic',      default_value='/imu/data',
                              description='IMU topic for navsat_transform_node'),
        DeclareLaunchArgument('gps_topic',      default_value='/imu/nav_sat_fix',
                              description='GPS fix topic for navsat_transform_node'),
        DeclareLaunchArgument('odom_frame',     default_value='odom',
                              description='Frame ID for the published global map'),
        DeclareLaunchArgument('map_resolution', default_value='0.2',
                              description='Voxel grid leaf size for map downsampling [m]'),
        OpaqueFunction(function=launch_setup),
    ])
