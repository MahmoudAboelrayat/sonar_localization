import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sonar_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_usv.yaml')
    vgicp_path = os.path.join(pkg_share, 'config', 'vgicp_rov.yaml')
    return LaunchDescription([

        #### Topics Bridges ####
        Node(
            package='sonar_localization',
            executable='dvl_bridge',
            name='dvl_bridge',
            output='screen',
            parameters=[{'sim': False, 'frame_id':'/nucleus_node/bottom_track_packets'}],
        ),
        Node(
            package='sonar_localization',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{'frame_id': 'imu_link_ned',
                         'add_noise':False,
                         'imu_topic': '/imu/data',
                         'output_topic': '/imu/data_noisy'}],),


        # Node(
        #     package='sonar_localization',
        #     executable='depth_bridge',
        #     name='depth_bridge',
        #     output='screen',
        #     parameters=[{'frame_id': 'Beckholmen','rel_alt_topic': '/mavros/global_position/rel_alt'}],),

            
        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config_path,{'use_sim_time': True}]
        ),
    #    Node(
    #        package='sonar_localization',
    #        executable='imu_preintegration',
    #        name='imu_preintegration',
    #        output='screen',
    #        parameters=[{'use_sim_time': True}]
    #    ),
        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0','0.0', '0', '0.0', 'base_link', 'imu_link_ned'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments=['0', '0', '0', '3.1415', '0', '0.0', 'base_link', 'gps'],
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
            name='base_link_to_sonar',
            arguments=['0.280', '0', '-1.120', '0.0', '0.436', '0.0', 'base_link', '3d_link'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_dvl',
            arguments=['-0.250', '0.0', '-1.120', '0.0', '0.0', '0.0', 'base_link', 'dvl_link'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_isss',
            arguments=['0.0', '0.0', '-1.070', '0.0', '0.0', '0.0', 'base_link', 'isss_link'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_acomm',
            arguments=['-0.350', '0.0', '-1.170', '0.0', '0.0', '0.0', 'base_link', 'acomm_link'],
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
        # Node(
        #     package='sonar_localization',
        #     executable='fastvgicp_odom',
        #     name='fastvgicp_odom',
        #     output='screen',
        #     parameters=[vgicp_path]
        # ),
        
        
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
        #         ('/gps/fix', '/gps/fix'),     # Map to your SBG topic
        #         ('imu', '/imu/data'),            # Map to your IMU topic
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





