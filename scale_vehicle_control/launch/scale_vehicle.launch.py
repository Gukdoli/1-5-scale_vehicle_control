from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scale_vehicle_control',
            executable='scale_vehicle_node',
            name='scale_vehicle_node',
            output='screen',
            parameters=[{
                'steering_offset': 518,
                'steering_range': 380,
                'max_speed': 250
            }]
        )
    ])
