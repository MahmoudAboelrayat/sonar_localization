import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sonar_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_sim.yaml')

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
            package='sonar_localization',
            executable='loopclosure_vgicp',
            name='loopclosure_vgicp',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'loop_vgicp_sim.yaml')]
        ),

       
        # rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'sim.rviz')]
        ),
    ])





