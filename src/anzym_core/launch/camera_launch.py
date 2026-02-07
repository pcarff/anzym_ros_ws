import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # USB Cam (RGB Color) - Arm Camera
    arm_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='arm_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0', 
            'framerate': 15.0,
            'image_width': 320,
            'image_height': 240,
            'pixel_format': 'yuyv2rgb',
            'frame_id': 'mono_link'
        }]
    )

    # Astra Camera RGB - Using usb_cam for better stability on Jetson
    astra_rgb_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera/color',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2', 
            'framerate': 15.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'frame_id': 'camera_color_frame'
        }]
    )

    # Astra Camera (Depth Only) - Using IncludeLaunchDescription with AnyLaunchDescriptionSource for XML
    astra_camera_package_dir = get_package_share_directory('astra_camera')
    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(astra_camera_package_dir, 'launch', 'astra.launch.xml')),
        launch_arguments={
            'enable_color': 'false', # Using usb_cam for color instead
            'enable_depth': 'true',
            'enable_ir': 'false'
        }.items()
    )
    
    return LaunchDescription([
        # Launch Astra Depth immediately to let it claim the device first
        astra_launch,
        # Delay Arm Camera slightly to stagger USB inrush
        TimerAction(period=5.0, actions=[arm_cam_node]),
        # Delay Astra RGB significantly to ensure OpenNI driver has settled
        TimerAction(period=10.0, actions=[astra_rgb_node])
    ])
