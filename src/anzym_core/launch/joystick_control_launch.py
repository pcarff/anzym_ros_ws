from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
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

        # Joy Node (Reads the physical joystick)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # Teleop Twist Joy (Converts joy -> cmd_vel)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'enable_button': -1, # Enable always (or set to your deadman switch index)
                'axis_linear.x': 1,  # Left stick vertical
                'scale_linear.x': 0.5,
                'axis_linear.y': 0,  # Left stick horizontal (Strafe)
                'scale_linear.y': 0.5,
                'axis_angular.yaw': 2, # Right stick horizontal
                'scale_angular.yaw': 1.0,
                'require_enable_button': False
            }]
        ),

        # Rosmaster Driver (Talks to the robot hardware)
        Node(
            package='anzym_core',
            executable='driver_node',
            name='rosmaster_driver',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'car_type': 2 # X3 Plus (from Rosmaster_Lib.py CARTYPE_X3_PLUS = 0x02)
            }]
        )
    ])
