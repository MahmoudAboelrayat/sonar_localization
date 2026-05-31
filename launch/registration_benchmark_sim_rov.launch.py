import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('sonar_localization')
    ekf_config_path   = os.path.join(pkg_share, 'config', 'ekf_sim_rov.yaml')
    bench_config_path = os.path.join(pkg_share, 'config', 'registration_benchmark_sim_rov.yaml')
    rviz_config_path  = os.path.join(pkg_share, 'rviz', 'sim_rov.rviz')

    method = LaunchConfiguration('method').perform(context).upper()

    angle = 90.0 * 3.14159265359 / 180.0
    init_x, init_y, init_z = 0.0, 0.0, 0.001

    return [

        #### Topic Bridges ####
        Node(
            package='sonar_localization',
            executable='dvl_bridge',
            name='dvl_bridge',
            output='screen',
            parameters=[{'sim': False, 'frame_id': 'BlueROV2_Heavy/dvl_link'}],
        ),
        Node(
            package='sonar_localization',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{
                'frame_id':  'BlueROV2_Heavy/base_link',
                'imu_topic': '/BlueROV2_Heavy/mavros/imu/data_raw',
            }],
        ),

        #### EKF Node ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            parameters=[ekf_config_path, {'use_sim_time': True}]
        ),

        #### TF ####
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_x_odom',
            arguments=[str(init_x), str(init_y), str(init_z),
                       str(angle), '0', '0',
                       'BlueROV2_Heavy/odom', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),

        #### Registration Benchmark ####
        Node(
            package='sonar_localization',
            executable='registration_benchmark',
            name='vgicp_odom_node',
            output='screen',
            parameters=[bench_config_path, {'registration.method': method}]
        ),

        #### RViz ####
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='VGICP',
                              description='Registration method: VGICP | GICP | ICP | NDT | ICPNL'),
        OpaqueFunction(function=launch_setup),
    ])
