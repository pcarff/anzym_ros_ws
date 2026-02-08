import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    ekf_config_path = os.path.join(get_package_share_directory('anzym_core'), 'config', 'ekf_params.yaml')
    use_ekf = LaunchConfiguration('use_ekf')
    
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

        DeclareLaunchArgument(
            'use_ekf',
            default_value='True',
            description='Enable Robot Localization (Sensor Fusion)'
        ),

        DeclareLaunchArgument(
            'use_cameras',
            default_value='True',
            description='Enable Camera Stack'
        ),

        # Driver Node (WITH EKF): Disable Internal TF
        Node(
            package='anzym_core',
            executable='driver_node',
            name='rosmaster_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'car_type': LaunchConfiguration('car_type'),
                'init_pose': [90.0, 135.0, 0.0, 0.0, 90.0, 180.0],
                'publish_tf': False
            }],
            condition=IfCondition(use_ekf)
        ),

        # Driver Node (WITHOUT EKF): Enable Internal TF
        Node(
            package='anzym_core',
            executable='driver_node',
            name='rosmaster_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'car_type': LaunchConfiguration('car_type'),
                'init_pose': [90.0, 135.0, 0.0, 0.0, 90.0, 180.0],
                'publish_tf': True
            }],
            condition=UnlessCondition(use_ekf)
        ),
        
        # Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            condition=IfCondition(use_ekf)
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
            ),
            condition=IfCondition(LaunchConfiguration('use_cameras'))
        )
    ])
