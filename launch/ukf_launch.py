import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sonar_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    ukf_config_path = os.path.join(pkg_share, 'config', 'ukf.yaml')
    return LaunchDescription([

        #### Topics Bridges ####
        Node(
            package='sonar_localization',
            executable='dvl_bridge',
            name='dvl_bridge',
            output='screen',
            parameters=[],
        ),

        Node(
            package='sonar_localization',
            executable='depth_bridge',
            name='depth_bridge',
            output='screen',
            parameters=[],),

        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config_path,{'use_sim_time': True}]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_config_path,{'use_sim_time': True}]
        ),

        #### UKF NODES ####
        Node(
                package='robot_localization',
                executable='ukf_node',
                name='ukf_local',
                output='screen',
                parameters=[ukf_config_path,{'use_sim_time': True}],
                remappings=[
                            ('odometry/filtered', 'odometry/filtered_ukf')]
            ),
            Node(
                package='robot_localization',
                executable='ukf_node',
                name='ukf_global',
                output='screen',
                parameters=[ukf_config_path,{'use_sim_time': True}],
                remappings=[
                 ('odometry/filtered', 'odometry/filtered_ukf')]
            ),

        #### Navsat #####
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[{'use_sim_time': True}, ekf_config_path],
            remappings=[
                ('/gps/fix', '/fix'),     # Map to your SBG topic
                ('imu', '/mavros/imu/data'),            # Map to your IMU topic
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),

        #### TFS ####
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0', '0.0', '0', '0.0', 'saabmarine/base_link', 'base_link'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments=['0', '0', '0', '0', '0', '0.0', 'saabmarine/base_link', 'gps'],
            parameters=[{'use_sim_time': True}]
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Beckholmen_tf',
            arguments=['0', '0', '0', '0.0', '0', '3.14159265359', 'world', 'Beckholmen'],
            parameters=[{'use_sim_time': True}]
        ),
            # Node(
            #     package='sonar_localization',
            #     executable='ekf_to_csv_logger',
            #     name='ekf_to_csv_logger',
            #     output='screen',
            #     parameters=[{'use_sim_time': True}]
            # )
    ])





