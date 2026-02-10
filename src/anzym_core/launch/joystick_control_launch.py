import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    pkg_anzym_core = get_package_share_directory('anzym_core')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device file'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Rosmaster serial port'
        ),
        DeclareLaunchArgument(
            'launch_driver',
            default_value='True',
            description='Whether to launch the local Rosmaster driver'
        ),

        # 1. Include the Robot Bringup (Drivers) - Conditionally
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_anzym_core, 'launch', 'bringup_launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('launch_driver')),
            launch_arguments={'port': LaunchConfiguration('port')}.items()
        ),

        # 2. Joy Node (Reads the physical joystick)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.15,
                'autorepeat_rate': 20.0,
            }]
        ),

        # 3. Teleop Twist Joy (Converts joy -> cmd_vel)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'enable_button': -1,
                'axis_linear.x': 1,
                'scale_linear.x': 0.5,
                'axis_linear.y': 0,
                'scale_linear.y': 0.5,
                'axis_angular.yaw': 2,
                'scale_angular.yaw': 0.5,
                'require_enable_button': False
            }]
        ),

        # 4. Arm Teleop Node (Converts joy -> joint_commands)
        Node(
            package='anzym_core',
            executable='arm_teleop_node',
            name='arm_teleop_node'
        )
    ])
