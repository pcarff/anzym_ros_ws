import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('anzym_description')
    
    # Path to xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'yahboomcar_X3plus.urdf.xacro')

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # joint_state_publisher (Fake joints for fixed links, often needed for full tree)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node
    ])
