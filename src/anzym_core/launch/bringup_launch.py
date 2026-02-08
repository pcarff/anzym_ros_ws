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
            default_value='/dev/rosmaster',
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
                'car_type': LaunchConfiguration('car_type'),
                # Set initial arm pose: [J1, J2, J3, J4, J5, Grip]
                # Default "folded" pose (Elbow bent 90 degrees forward from up)
                'init_pose': [90.0, 135.0, 0.0, 0.0, 90.0, 180.0]
            }]
        ),

        # YDLidar Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
            ),
            launch_arguments={'params_file': os.path.join(get_package_share_directory('anzym_core'), 'config', 'ydlidar.yaml')}.items()
        ),
        
        # Robot State Publisher (URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('anzym_description'), 'launch', 'description_launch.py')
            )
        ),

        # Camera Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('anzym_core'), 'launch', 'camera_launch.py')
            )
        )
    ])
