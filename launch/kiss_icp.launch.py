from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kiss_icp',
            executable='kiss_icp_node',
            name='kiss_icp',
            output='screen',
            parameters=[{
                'lidar_odom_frame': 'odom',
                'child_frame': 'sam_auv_v1/sonar_link', 
                
                'voxel_size': 0.5,
                'max_range': 15.0,        
                'min_range': 1.0,           
                'deskew': False,            
                'max_points_per_voxel': 20, 
                

                'publish_odom_tf': False,   
            }],
            remappings=[
                ('pointcloud_topic', '/sonar/point_cloud_local'),
            ]
        ),
    ])