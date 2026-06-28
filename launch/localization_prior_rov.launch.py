import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    pkg_share       = get_package_share_directory('sonar_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_loop.yaml')
    slam_type = LaunchConfiguration('slam_type').perform(context).lower()

    if slam_type == 'loose':
        loc_config_path = os.path.join(pkg_share, 'config', 'loop_vgicp_rov_june.yaml')
    else:  # 'tight' or default
        loc_config_path = os.path.join(pkg_share, 'config', 'localization_imu_pre_rov.yaml')
    navsat_config   = os.path.join(pkg_share, 'config', 'navsat.yaml')

    rviz_flag  = LaunchConfiguration('rviz').perform(context).lower()     in ('true', '1', 'yes')
    use_dock   = LaunchConfiguration('dock').perform(context).lower()     in ('true', '1', 'yes')
    sim_time   = LaunchConfiguration('sim_time').perform(context).lower() in ('true', '1', 'yes')
    dvl_type   = LaunchConfiguration('dvl_type').perform(context).lower()
    map_file   = LaunchConfiguration('map_file').perform(context)
    ns         = LaunchConfiguration('namespace').perform(context)
    # Helper: prefix frame name with namespace when one is set
    def f(frame): return f'{ns}/{frame}' if ns else frame

    init_x       = float(LaunchConfiguration('init_x').perform(context))
    init_y       = float(LaunchConfiguration('init_y').perform(context))
    init_z       = float(LaunchConfiguration('init_z').perform(context))
    init_yaw     = float(LaunchConfiguration('init_yaw').perform(context))
    init_roll    = float(LaunchConfiguration('init_roll').perform(context))
    init_pitch   = float(LaunchConfiguration('init_pitch').perform(context))
    min_keyframes = int(LaunchConfiguration('min_keyframes').perform(context))

    return [

        #### EKF Nodes ####
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            namespace=ns,
            output='screen',
            parameters=[ekf_config_path, {'odom_frame': f('ekf_odom'), 'base_link_frame': f('base_link'),'world_frame': f('ekf_odom'), 'use_sim_time': sim_time}],
            remappings=[('odometry/filtered', 'odometry/ekf_local')]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            namespace=ns,
            output='screen',
            parameters=[ekf_config_path, {'odom_frame': f('ekf_odom'), 'base_link_frame': f('base_link'), 'use_sim_time': sim_time}],
        ),

        #### Static TF ####
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Beckholmen_tf',
            arguments=['0', '0', '0', '0.0', '0', '3.14159265359', 'world', 'Beckholmen'],
            parameters=[{'use_sim_time': sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar',
            namespace=ns,
            arguments=['0.165', '0.037', '-0.114', '0.0', '-0.3926991', '0.0',
                       f('base_link'), 'sonar_link'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            namespace=ns,
            arguments=['0.165', '-0.052', '-0.114', '1.5707963', '0.0', '1.1780972',
                       f('base_link'), f('hd_camera_link')],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_dvl',
            namespace=ns,
            arguments=['0.0', '-0.106', '0.265', '0.0', '0.0', '0.0',
                       f('base_link'), 'dvl_link'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gss',
            namespace=ns,
            arguments=['0.0', '-0.106', '-0.2', '0.0', '0.0', '0.0',
                       f('base_link'), f('gnss')],
            output='screen',
        ),

        #### Map TF (odom_tf.py listens to /initialpose to update this) ####
        Node(
            package='sonar_localization',
            executable='odom_tf',
            name='map_tf',
            namespace=ns,
            output='screen',
            parameters=[{
                'rviz_is_ned':   False,
                'parent_frame': '/world',
                'child_frame':  '/icp_map',
                'use_sim_time': sim_time,
                'init_x':       init_x,
                'init_y':       init_y,
                'init_z':       init_z,
                'init_yaw':       init_yaw,
                'init_roll':      init_roll,
                'init_pitch':     init_pitch,
                'lock_roll_pitch': True,
                'lock_z':          True,
            }]
        ),

        #### Prior map publisher ####
        Node(
            package='sonar_localization',
            executable='prior_map_publisher',
            name='prior_map_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'prior_map_path':        map_file,
                'map_topic':             '/prior_map',
                'map_frame':             '/world',
                'prior_map_resolution':  0.05,
                'is_ned':                False,
                'use_sim_time':          sim_time,
            }]
        ),

        #### SLAM node (tight = localization_imu_pre, loose = loopclosure_vgicp) ####
        Node(
            package='sonar_localization',
            executable='localization_imu_pre' if slam_type != 'loose' else 'loopclosure_vgicp',
            name='localization_imu_pre'        if slam_type != 'loose' else 'odom_vgicp',
            namespace=ns,
            output='screen',
            parameters=[loc_config_path, {'use_sim_time': sim_time}]
        ),

        #### Map matcher (aligns SLAM map → global map, publishes /initialpose) ####
        Node(
            package='sonar_localization',
            executable='map_matcher',
            name='map_matcher',
            namespace=ns,
            output='screen',
            parameters=[{
                'global_map_topic':  '/prior_map',  # pre-built map published by SLAM node
                'slam_map_topic':    'vgicp_sub_map',
                'initialpose_topic': '/initialpose',
                'child_frame':       '/icp_map',
                'period':            3.0,
                'min_keyframes':     min_keyframes,
                'voxel_size':        0.3,
                'fitness_threshold': 0.8,
                'max_corr_dist':     3.0,
                'is_ned':            True,
                'use_ndt':           True,
                'ndt_resolution':    1.5,
                'ndt_step_size':     1.0,
                'ndt_epsilon':       0.01,
                'ndt_max_iter':      100,
                'vgicp_threads':     4,
                'vgicp_max_iter':    200,
                'vgicp_resolution':  0.3,
                'use_sim_time':      sim_time,
            }]
        ),

        #### RViz ####
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'prior.rviz')],
            parameters=[{'use_sim_time': sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),

        #### Dry dock model ####
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='dry_dock_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(
                    os.path.join(pkg_share, 'description', 'dry_dock.urdf')
                ).read(),
                'frame_id': 'Beckholmen',
            }],
            condition=IfCondition(LaunchConfiguration('dock'))
        ),

        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform_node',
        #     output='screen',
        #     parameters=[navsat_config, {'use_sim_time': sim_time}],
        #     remappings=[
        #         ('imu',                '/dvl/imu'),
        #         ('gps/fix',            'fix'),
        #         ('odometry/filtered',  'odometry/filtered'),
        #     ],
        # ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS2 namespace for all nodes (empty = no namespace)'),
        DeclareLaunchArgument('slam_type', default_value='tight',
                              description='tight = localization_imu_pre | loose = loopclosure_vgicp'),
        DeclareLaunchArgument('dvl_type',  default_value='nucleus',
                              description='dvl model: nucleus / waterlinked / sim'),
        DeclareLaunchArgument('map_file',   default_value='/home/mahmoud/thesis_ws/src/sonar_localization/maps/prior_map.pcd',
                              description='absolute path to the prior map .pcd file'),
        DeclareLaunchArgument('init_x',     default_value='2.17055',  description='initial TF x [m]'),
        DeclareLaunchArgument('init_y',     default_value='0.359599', description='initial TF y [m]'),
        DeclareLaunchArgument('init_z',     default_value='0.0',      description='initial TF z [m]'),
        DeclareLaunchArgument('init_yaw',   default_value='0.0',      description='initial TF yaw [deg]'),
        DeclareLaunchArgument('init_roll',     default_value='0.0', description='initial TF roll [rad]'),
        DeclareLaunchArgument('init_pitch',    default_value='0.0', description='initial TF pitch [rad]'),
        DeclareLaunchArgument('min_keyframes', default_value='2',   description='keyframes to collect before map matching starts'),
        DeclareLaunchArgument('rviz',      default_value='true',
                              description='open rviz window'),
        DeclareLaunchArgument('sim_time',  default_value='true',
                              description='use sim_time'),
        DeclareLaunchArgument('dock',      default_value='false',
                              description='show dry dock model'),
        OpaqueFunction(function=launch_setup),
    ])
