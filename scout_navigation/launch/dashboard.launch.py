"""Operator dashboard: rosbridge for telemetry, MJPEG for the camera, and the page itself on port 8000."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WEB = os.path.join(get_package_share_directory('scout_navigation'), 'web')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('usb_camera', default_value='false',
                              description='Publish the real forward camera; off in simulation'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
             parameters=[{'port': 9090, 'use_sim_time': LaunchConfiguration('use_sim_time')}]),

        Node(package='web_video_server', executable='web_video_server', output='screen',
             parameters=[{'port': 8080, 'default_stream_type': 'ros_compressed'}]),

        Node(package='usb_cam', executable='usb_cam_node_exe', name='front_camera', output='screen',
             condition=IfCondition(LaunchConfiguration('usb_camera')),
             parameters=[{'video_device': '/dev/video0', 'image_width': 640, 'image_height': 360,
                          'framerate': 15.0, 'pixel_format': 'yuyv'}],
             remappings=[('image_raw', 'front_camera/image')]),

        ExecuteProcess(cmd=['python3', '-m', 'http.server', '8000', '--directory', WEB],
                       output='screen'),
    ])
