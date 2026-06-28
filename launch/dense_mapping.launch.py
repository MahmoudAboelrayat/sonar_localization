import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    pkg_share     = get_package_share_directory('sonar_localization')
    ekf_config    = os.path.join(pkg_share, 'config', 'ekf_usv.yaml')
    sim_time      = LaunchConfiguration('sim_time').perform(context).lower() in ('true', '1', 'yes')
    surface_method= LaunchConfiguration('surface').perform(context).lower()   # 'gp3' or 'poisson'

    return [

        #### TF ####
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar',
            arguments=['0.38', '0.08', '-0.525', '0.0', '0.52', '3.14159', 'base_link', 'usv/sonar_link'],
            parameters=[{'use_sim_time': sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '3.1415', 'base_link', 'imu_link_ned'],
            parameters=[{'use_sim_time': sim_time}]
        ),

        #### EKF ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': sim_time}],
        ),

        #### Dense Mapping ####
        Node(
            package='sonar_localization',
            executable='dense_mapping',
            name='dense_mapping_node',
            output='screen',
            parameters=[{
                'pc_topic':            'usv/point_cloud',
                'odom_topic':          '/imu/odometry',
                'map_topic':           '/dense_map',
                'odom_frame':          'odom',
                'map_resolution':      0.02,       # 2 cm voxel deduplication
                'publish_every':       10,
                'odom_warmup':         10,
                'max_range':           -1.0,        # -1 = no limit
                'max_z':               -1.0,
                'save_on_shutdown':    True,
                'save_pcd_path':       '/tmp/dense_map.pcd',
                'save_ply_path':       '/tmp/dense_mesh.ply',
                # Surface reconstruction
                'reconstruct_surface': True,
                'surface_method':      surface_method,  # 'gp3' or 'poisson'
                'reconstruct_every':   50,
                'mls_radius':          0.2,
                'gp3_search_radius':   1.0,
                'gp3_mu':              3.0,
                'gp3_max_nn':          200,
                'poisson_depth':       8,
                'min_pts_for_surface': 200,
                'normal_k':            20,
                # Outlier removal (per scan, before accumulation)
                'use_sor':             True,
                'sor_k':               10,
                'sor_std_thresh':      1.0,
                'use_ror':             True,
                'ror_min_neighbors':   5,
                'ror_radius':          0.5,
                # Sonar extrinsics
                'base2sonar_tx':       0.38,
                'base2sonar_ty':       0.08,
                'base2sonar_tz':      -0.525,
                'base2sonar_r':        180.0,
                'base2sonar_p':         25.0,
                'base2sonar_y':          0.0,
                'use_sim_time':        sim_time,
            }],
        ),

        #### RViz ####
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'usv_mapping.rviz')],
            parameters=[{'use_sim_time': sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz',       default_value='true',
                              description='open RViz window'),
        DeclareLaunchArgument('sim_time',   default_value='false',
                              description='use sim_time'),
        DeclareLaunchArgument('surface',    default_value='gp3',
                              description='surface reconstruction method: gp3 or poisson'),
        OpaqueFunction(function=launch_setup),
    ])