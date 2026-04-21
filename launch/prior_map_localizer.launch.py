import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dead_reckoning')
    ekf_config_path        = os.path.join(pkg_share, 'config', 'ekf_global_updated.yaml')
    localizer_config_path  = os.path.join(pkg_share, 'config', 'prior_map_localizer.yaml')
    rviz_config_path       = os.path.join(pkg_share, 'rviz', 'sim_prior.rviz')

    # angle  = 90.0 * 3.14159265359 / 180.0
    # init_x = 0.0
    # init_y = 0.0
    # init_z = -0.03
    # angle = 53.081 * 3.14159265359/180.0
    # init_x = 0.125
    # init_y = -0.042
    # init_z = -0.000
    angle = 55.43 * 3.14159265359/180.0
    init_x = 0.763
    init_y = 1.005
    init_z = -0.000
    return LaunchDescription([

        #### Topics Bridges ####
        Node(
            package='dead_reckoning',
            executable='dvl_bridge',
            name='dvl_bridge',
            output='screen',
            parameters=[{'sim': False, 'frame_id': 'BlueROV2_Heavy/dvl_link'}],
        ),

        Node(
            package='dead_reckoning',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{'frame_id': 'BlueROV2_Heavy/base_link',
                         'imu_topic': '/BlueROV2_Heavy/mavros/imu/data_raw'}],
        ),

        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            parameters=[ekf_config_path, {'use_sim_time': True}],
        ),

        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_x_odom',
            arguments=[str(init_x), str(init_y), str(init_z),
                       str(angle), '0', '0',
                       'BlueROV2_Heavy/odom', 'odom'],
            parameters=[{'use_sim_time': True}],
        ),

        # Prior map localizer
        Node(
            package='dead_reckoning',
            executable='prior_map_localizer',
            name='prior_map_localizer',
            output='screen',
            parameters=[localizer_config_path, {'use_sim_time': True}],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
        ),
    ])
