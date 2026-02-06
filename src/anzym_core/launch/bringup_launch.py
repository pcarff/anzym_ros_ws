import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB1',
            description='Rosmaster serial port'
        ),
        
        DeclareLaunchArgument(
            'car_type',
            default_value='1', # X3
            description='Car type: 1=X3, 2=X3 Plus, 4=R2'
        ),

        # Rosmaster Driver (The core brain talking to the hardware)
        Node(
            package='anzym_core',
            executable='driver_node',
            name='rosmaster_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'car_type': LaunchConfiguration('car_type')
            }]
        ),

        # YDLidar Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
            )
        )
    ])
