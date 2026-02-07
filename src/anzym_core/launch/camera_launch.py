import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # USB Cam (RGB Color)
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
            'framerate': 30.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv'  # Common format for webcams
        }]
    )

    # Astra Camera (Depth Only) - Using IncludeLaunchDescription with AnyLaunchDescriptionSource for XML
    astra_camera_package_dir = get_package_share_directory('astra_camera')
    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(astra_camera_package_dir, 'launch', 'astra.launch.xml')),
        launch_arguments={
            'enable_color': 'false',
            'enable_depth': 'true',
            'enable_ir': 'false'
        }.items()
    )
    
    return LaunchDescription([
        usb_cam_node,
        astra_launch
    ])

