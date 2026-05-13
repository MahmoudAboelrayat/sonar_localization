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
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_loop.yaml')

    rviz_flag  = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1', 'yes')
    full_bag  = LaunchConfiguration('full_bag').perform(context).lower() in ('true', '1', 'yes')
    gicp_backend = LaunchConfiguration('gicp').perform(context).lower()  # 'fast' or 'small'
    sim_time = LaunchConfiguration('sim_time').perform(context).lower() in ('true', '1', 'yes')
    dvl_type = LaunchConfiguration('dvl_type').perform(context).lower()

    if full_bag:
        init_x, init_y, init_z, init_yaw = 52.670, -2.4, 0.0, -0.096
        if gicp_backend == 'small':
            vgicp_config_path = os.path.join(pkg_share, 'config', 'loop_small_vgicp_rov.yaml')
        else:
            vgicp_config_path = os.path.join(pkg_share, 'config', 'loop_vgicp_rov_full.yaml')
    else:
        init_x, init_y, init_z, init_yaw = 38.529, -2.881, -1.5, -3.031
        if gicp_backend == 'small':
            vgicp_config_path = os.path.join(pkg_share, 'config', 'loop_small_vgicp_rov.yaml')
        else:
            vgicp_config_path = os.path.join(pkg_share, 'config', 'loop_vgicp_rov.yaml')

    icp_executable = 'loopclosure_samll_vgicp' if gicp_backend == 'small' else 'loopclosure_vgicp'

    return [

        #### Topics Bridges ####
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensors_bridge.launch.py')
            ),
            launch_arguments={
                'dvl_type': dvl_type,
                'dvl_frame': 'saabmarine/dvl_frame',
                'depth_frame': 'icp_map',
                'relative_depth': 'true',
                'ned': 'true' if full_bag else 'false',
            }.items(),
        ),

            
        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config_path,{'use_sim_time': sim_time}],
            remappings=[
                ('odometry/filtered', 'odometry/ekf_local')
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_config_path,{'use_sim_time': sim_time}]
        ),


        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0','0.0', '0', '3.14159265359', 'saabmarine/base_link', 'base_link'],
            parameters=[{'use_sim_time': sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments=['0', '0', '0', '3.1415', '0', '0.0', 'saabmarine/base_link', 'gps'],
            parameters=[{'use_sim_time': sim_time}]
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Beckholmen_tf',
            arguments=['0', '0', '0', '0.0', '0', '3.14159265359', 'world', 'Beckholmen'],
            parameters=[{'use_sim_time': sim_time}]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar',
            arguments=['0.220', '0', '-0.160', '0.0', '-0.524', '0.0', 'saabmarine/base_link', 'saabmarine/sonar_link'],
            parameters=[{'use_sim_time': sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_dvl',
            arguments=['-0.150', '-0.150', '0.150', '0.0', '0.0', '0.0', 'saabmarine/base_link', 'saabmarine/dvl_frame'],
            parameters=[{'use_sim_time': sim_time}]
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='Beckholmen_ekf',
        #     arguments=['0', '0', '0', '0.0', '0', '0.0', 'Beckholmen', 'icp_map'],
        #     parameters=[{'use_sim_time': True}]
        # ),

        ############ cropped bag
        # Node(
        #     package='sonar_localization',
        #     executable='odom_tf',
        #     name='map_tf',
        #     output='screen',
        #     parameters=[{'parent_frame': 'Beckholmen', 'child_frame': 'icp_map', 'use_sim_time': True, 'init_x': 37.77, 'init_y': -2.6, 'init_z': -1.5, 'init_yaw': -3.1415,'init_roll':0.0,'init_pitch':0.0}]
        # ),

        ############ full bag
         Node(
            package='sonar_localization',
            executable='odom_tf',
            name='map_tf',
            output='screen',
            parameters=[{'parent_frame': 'Beckholmen', 'child_frame': 'icp_map', 'use_sim_time': sim_time, 'init_x': init_x, 'init_y': init_y, 'init_z': init_z, 'init_yaw': init_yaw,'init_roll':0.0,'init_pitch':0.0}]
        ),

        # vgicp odometry
        Node(
            package='sonar_localization',
            executable=icp_executable,
            name='odom_vgicp',
            output='screen',
            parameters=[vgicp_config_path]
        ),
        
        # rviz

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'rov_dry_dock.rviz')],
            parameters=[{'use_sim_time': sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))
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
                'frame_id': 'Beckholmen',   # publishes my_link relative to this frame
            }],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),


        ### Navsat #####
        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}, ekf_config_path],
        #     remappings=[
        #         ('/gps/fix', '/fix'),     # Map to your SBG topic
        #         ('imu', '/mavros/imu/data'),            # Map to your IMU topic
        #         ('odometry/filtered', '/odometry/filtered')
        #     ]
        # ),
        # Node(
        #     package='sonar_localization',
        #     executable='ekf_to_csv_logger',
        #     name='ekf_to_csv_logger',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )

    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dvl_type', default_value='waterlinked',
                              description='dvl model: (nucleus) or (waterlinked) or (sim)'),
        DeclareLaunchArgument('full_bag', default_value='true',
                              description='true = full bag, false = cropped bag'),
        DeclareLaunchArgument('gicp', default_value='fast',
                              description='gicp backend: fast (fast_gicp) or small (small_gicp)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='open rviz widow or not'),
        DeclareLaunchArgument('sim_time', default_value='true',
                              description='use sim_time or not'),
        OpaqueFunction(function=launch_setup),
    ])





