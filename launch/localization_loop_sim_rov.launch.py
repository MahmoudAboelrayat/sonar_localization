import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sonar_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_sim_rov.yaml')
    vgicp_config_path = os.path.join(pkg_share, 'config', 'loop_vgicp_sim_rov.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'sim_rov.rviz')
    # angle = 55.43 * 3.14159265359/180.0
    # init_x = 0.763
    # init_y = 1.005
    # init_z = -0.000

    angle = 53.081 * 3.14159265359/180.0
    # init_x = 0.125
    # init_y = -0.042
    # init_z = -0.000

    # 0.166, 0.013, 0.001
    init_x = 0.166
    init_y = 0.013
    init_z = 0.001
    return LaunchDescription([

        #### Topics Bridges ####
        Node(
            package='sonar_localization',
            executable='dvl_bridge',
            name='dvl_bridge',
            output='screen',
            parameters=[{'sim': False, 'frame_id':'BlueROV2_Heavy/dvl_link'}],
        ),

        Node(
            package='sonar_localization',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{'frame_id':'BlueROV2_Heavy/base_link','imu_topic':'/BlueROV2_Heavy/mavros/imu/data_raw'}],),
            
        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            # output='screen',
            parameters=[ekf_config_path,{'use_sim_time': True}]
            # remappings=[
            #     ('odometry/filtered', 'odometry/ekf_local')
            # ]
        ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_global',
        #     # output='screen',
        #     parameters=[ekf_config_path,{'use_sim_time': True}]
        # ),

        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_x_odom',
            arguments=[str(init_x), str(init_y), str(init_z), str(angle), '0', '0', 'BlueROV2_Heavy/odom', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),

        # vgicp odometry
        
        Node(
            package='sonar_localization',
            executable='loopclosure_vgicp',
            name='loopclosure_vgicp',
            output='screen',
            parameters=[vgicp_config_path]
        ),

       
        # rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])





