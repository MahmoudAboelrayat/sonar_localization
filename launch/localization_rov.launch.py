import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('sonar_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    vgicp_config_path = os.path.join(pkg_share, 'config', 'loop_vgicp_rov.yaml')
    return LaunchDescription([

        DeclareLaunchArgument('dvl_type', default_value='waterlinked',
                              description='dvl model: (nucleus) or (waterlinked) or (sim)'),

        #### Topics Bridges ####
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensors_bridge.launch.py')
            ),
            launch_arguments={
                'dvl_type': LaunchConfiguration('dvl_type'),
                'dvl_frame': 'saabmarine/dvl_frame',
                'depth_frame': 'Beckholmen',
            }.items(),
        ),

            
        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config_path,{'use_sim_time': True}],
            remappings=[
                ('odometry/filtered', 'odometry/ekf_local')
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_config_path,{'use_sim_time': True}]
        ),


        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0','0.0', '0', '3.14159265359', 'saabmarine/base_link', 'base_link'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments=['0', '0', '0', '3.1415', '0', '0.0', 'saabmarine/base_link', 'gps'],
            parameters=[{'use_sim_time': True}]
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Beckholmen_tf',
            arguments=['0', '0', '0', '0.0', '0', '3.14159265359', 'world', 'Beckholmen'],
            parameters=[{'use_sim_time': True}]),

            Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Beckholmen_tf',
            arguments=['0', '0', '0', '0.0', '0', '0.0', 'Beckholmen','icp_map'],
            parameters=[{'use_sim_time': True}]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar',
            arguments=['0.220', '0', '-0.160', '0.0', '-0.524', '0.0', 'saabmarine/base_link', 'saabmarine/sonar_link'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_dvl',
            arguments=['-0.150', '-0.150', '0.150', '0.0', '0.0', '0.0', 'saabmarine/base_link', 'saabmarine/dvl_frame'],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Beckholmen_tf',
            arguments=['0', '0', '0', '0.0', '0', '0.0', 'Beckholmen', 'ekf_odom'],
            parameters=[{'use_sim_time': True}]
        ),

        # vgicp odometry
        Node(
            package='sonar_localization',
            executable='fastvgicp_odom',
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
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'rov.rviz')],
                parameters=[{'use_sim_time': True}]
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

    ])





