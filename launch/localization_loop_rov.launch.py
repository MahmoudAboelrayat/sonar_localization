import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dead_reckoning')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    vgicp_config_path = os.path.join(pkg_share, 'config', 'loop_vgicp_rov.yaml')
    return LaunchDescription([

        #### Topics Bridges ####
        Node(
            package='dead_reckoning',
            executable='dvl_bridge',
            name='dvl_bridge',
            output='screen',
            parameters=[{'sim': False, 'frame_id':'saabmarine/dvl_frame'}],
        ),

        Node(
            package='dead_reckoning',
            executable='depth_bridge',
            name='depth_bridge',
            output='screen',
            parameters=[{'frame_id': 'map','rel_alt_topic': '/mavros/global_position/rel_alt','relative_depth':True,"ned": False}],),

            
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

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='Beckholmen_ekf',
        #     arguments=['0', '0', '0', '0.0', '0', '0.0', 'Beckholmen', 'map'],
        #     parameters=[{'use_sim_time': True}]
        # ),

        Node(
            package='dead_reckoning',
            executable='odom_tf',
            name='map_tf',
            output='screen',
            parameters=[{'parent_frame': 'Beckholmen', 'child_frame': 'map', 'use_sim_time': True, 'init_x': 37.77, 'init_y': -2.6, 'init_z': -1.5, 'init_yaw': -3.10}]
        ),

        # vgicp odometry
        Node(
            package='dead_reckoning',
            executable='loopclosure_vgicp',
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
                'frame_id': 'Beckholmen',   # publishes my_link relative to this frame
            }]
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
        #     package='dead_reckoning',
        #     executable='ekf_to_csv_logger',
        #     name='ekf_to_csv_logger',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )

    ])





