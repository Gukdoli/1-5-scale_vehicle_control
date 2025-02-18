from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Vehicle control node
    vehicle_node = Node(
        package='scale_vehicle_control',
        executable='scale_vehicle_node',
        name='scale_vehicle_node',
        output='screen',
        parameters=[{
            'steering_offset': 518,
            'steering_range': 380,
            'max_steering': 420,
            'max_speed': 250,
            'max_speed_step': 15
        }],
        remappings=[
            ('/vehicle_control', '/scale_vehicle_topic'),
        ]
    )
    
    # GPS launch
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ublox_gps'), 'launch'),
            '/ublox_gps_node_zedf9p-launch.py'
        ])
    )

    return LaunchDescription([
        vehicle_node,
        gps_launch
    ])
