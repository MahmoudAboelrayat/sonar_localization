import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dead_reckoning')
    lio_pkg = get_package_share_directory('lio_sam')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_sim.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'sim.rviz')
    lio_parameter_file = os.path.join(lio_pkg, 'config', 'sonar.yaml')
    lio_rviz = os.path.join(lio_pkg,'config', 'rviz2.rviz')
    return LaunchDescription([

        #### Topics Bridges ####
        Node(
            package='dead_reckoning',
            executable='dvl_bridge',
            name='dvl_bridge',
            output='screen',
            parameters=[],
        ),

        Node(
            package='dead_reckoning',
            executable='depth_bridge',
            name='depth_bridge',
            output='screen',
            parameters=[],),

        
        Node(
            package='dead_reckoning',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[],),
            
        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            arguments=['--ros-args', '--log-level', 'tf2_buffer:=ERROR'],
            # output='screen',
            parameters=[ekf_config_path,{'use_sim_time': True}],
            remappings=[
                ('odometry/filtered', 'odometry/ekf_local')
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            arguments=['--ros-args', '--log-level', 'tf2_buffer:=ERROR'],
            # output='screen',
            parameters=[ekf_config_path,{'use_sim_time': True}]
        ),

        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_x_odom',
            arguments=['0', '0', '0','1.57', '0', '0', 'sam_auv_v1/odom', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),
    
        # vgicp odometry
        
        Node(
            package='dead_reckoning',
            executable='fastvgicp_odom',
            name='fastvgicp_odom',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'vgicp_sim.yaml')],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[lio_parameter_file],
            output='screen'
            ),

        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[lio_parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            remappings=[
            ('lio_sam/mapping/odometry', '/vgicp_odom'),
                ],
            parameters=[lio_parameter_file],
            output='screen'
        ),
       
        # rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # output='screen',
            arguments=['-d', lio_rviz]
        ),
    ])





