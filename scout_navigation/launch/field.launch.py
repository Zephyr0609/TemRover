"""Whole field experiment in one launch: sensor drivers, navigator, CAN bridge, IMU and dashboard."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_SHARE = get_package_share_directory('scout_navigation')
LAUNCH = os.path.join(PACKAGE_SHARE, 'launch')
CONFIG = os.path.join(PACKAGE_SHARE, 'config')


def include(path, arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=arguments.items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value=os.path.join(CONFIG, 'survey.yaml'),
                              description='Site parameters layered over survey.yaml'),
        DeclareLaunchArgument('payload', default_value=os.path.join(CONFIG, 'payload.yaml')),
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('gnss', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('gnss_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),

        include(os.path.join(LAUNCH, 'hardware.launch.py'),
                {'site': LaunchConfiguration('site'), 'payload': LaunchConfiguration('payload'),
                 'imu': LaunchConfiguration('imu')}),
        include(os.path.join(LAUNCH, 'dashboard.launch.py'), {'usb_camera': LaunchConfiguration('camera')}),

        Node(package='ublox_gps', executable='ublox_gps_node', name='gnss', output='screen',
             condition=IfCondition(LaunchConfiguration('gnss')),
             parameters=[os.path.join(CONFIG, 'ublox_rover.yaml'), {'device': LaunchConfiguration('gnss_port')}],
             remappings=[('fix', 'gps/fix'), ('fix_velocity', 'gps/fix_velocity')]),

        # Slamtec's own S2 launch, resolved only when the lidar is enabled
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_s2_launch.py'])),
            condition=IfCondition(LaunchConfiguration('lidar')),
            launch_arguments={'serial_port': LaunchConfiguration('lidar_port'),
                              'frame_id': 'lidar_link'}.items()),
    ])
