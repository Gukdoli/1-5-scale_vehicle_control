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
            'max_speed': 250
        }]
    )

    # LIDAR launch
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),
            '/sllidar_launch.py'
        ])
    )

    # GPS node
    gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[{
            'device': '/dev/ttyACM0',
            'frame_id': 'gps',
            'baudrate': 115200,
        }]
    )

    return LaunchDescription([
        vehicle_node,
        sllidar_launch,
        gps_node
    ])
