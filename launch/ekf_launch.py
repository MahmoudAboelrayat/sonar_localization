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
    ukf_config_path = os.path.join(pkg_share, 'config', 'ukf.yaml')

    return LaunchDescription([

        DeclareLaunchArgument('dvl_type', default_value='waterlinked',
                              description='dvl model: (nucleus) or (waterlinked) or (sim)'),

        #### Topics Bridges ####
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensors_bridge.launch.py')
            ),
            launch_arguments={'dvl_type': LaunchConfiguration('dvl_type')}.items(),
        ),

        
        Node(
            package='sonar_localization',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[],),
            
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

        #### UKF NODES ####
        Node(
                package='robot_localization',
                executable='ukf_node',
                name='ukf_local',
                output='screen',
                parameters=[ukf_config_path,{'use_sim_time': True}],
                remappings=[
                            ('odometry/filtered', 'odometry/filtered_ukf_local')]
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

        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_x_odom',
            arguments=['0', '0', '0','1.57', '0', '0', 'sam_auv_v1/odom', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_imu',
        #     arguments=['0', '0', '0','3.14159265359', '0', '3.14159265359', 'saabmarine/base_link', 'base_link'],
        #     parameters=[{'use_sim_time': True}]
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_gps',
        #     arguments=['0', '0', '0', '3.1415', '0', '0.0', 'saabmarine/base_link', 'gps'],
        #     parameters=[{'use_sim_time': True}]
        # ),
        #  Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='Beckholmen_tf',
        #     arguments=['0', '0', '0', '0.0', '0', '3.14159265359', 'world', 'Beckholmen'],
        #     parameters=[{'use_sim_time': True}]
        # ),
        # Node(
        #     package='sonar_localization',
        #     executable='ekf_to_csv_logger',
        #     name='ekf_to_csv_logger',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )

        # ros2 run tf2_ros static_transform_publisher --x 0.6739 --y  0 --z 0.0777 --roll 0 --pitch 0 --yaw 0 --frame-id sam_auv_v1/base_link --child-frame-id sam_auv_v1/3d_sonar

    ])





