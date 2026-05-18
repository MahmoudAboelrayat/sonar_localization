from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os

def generate_launch_description():
    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')
    log_output = LaunchConfiguration('log_output')
    fcu_protocol = LaunchConfiguration('fcu_protocol')
    respawn_mavros = LaunchConfiguration('respawn_mavros')


    # MAVROS package paths  
    mavros_pkg_dir = get_package_share_directory('mavros')  
    mavros_launch_file = os.path.join(mavros_pkg_dir, 'launch', 'node.launch')  # XML file  
    pluginlists_yaml = os.path.join(mavros_pkg_dir, 'launch', 'px4_pluginlists.yaml')  
    config_yaml = os.path.join(mavros_pkg_dir, 'launch', 'px4_config.yaml')  

    return LaunchDescription([  
        # Declare launch arguments  
        DeclareLaunchArgument('namespace', default_value='mavros', description='Robot namespace'),  
        DeclareLaunchArgument('fcu_url', default_value='udp-b://0.0.0.0:14550@14550'),  
        DeclareLaunchArgument('gcs_url', default_value=''),  
        DeclareLaunchArgument('tgt_system', default_value='1'),  
        DeclareLaunchArgument('tgt_component', default_value='1'),  
        DeclareLaunchArgument('log_output', default_value='screen'),  
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),  
        DeclareLaunchArgument('respawn_mavros', default_value='false'),  

        # Group under namespace  
        GroupAction([  
            IncludeLaunchDescription(  
                XMLLaunchDescriptionSource(mavros_launch_file),  
                launch_arguments={  
                    'pluginlists_yaml': pluginlists_yaml,  
                    'config_yaml': config_yaml,  
                    'fcu_url': fcu_url,  
                    'gcs_url': gcs_url,  
                    'tgt_system': tgt_system,  
                    'tgt_component': tgt_component,  
                    'log_output': log_output,  
                    'fcu_protocol': fcu_protocol,  
                    'respawn_mavros': respawn_mavros,  
                    'namespace': namespace  
                }.items()  
            )  
        ])  
    ])  