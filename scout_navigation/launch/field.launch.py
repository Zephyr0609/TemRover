"""Whole field experiment in one launch: sensor drivers, navigator, CAN bridge, IMU and dashboard."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
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
        # The team's LG290P bridge: finds the receiver's port, feeds it NTRIP corrections, serves its NMEA on TCP
        DeclareLaunchArgument('rtk_bridge', default_value=os.path.expanduser('~/ros2_ws/rtk_gps_bridge.py')),
        DeclareLaunchArgument('nmea_port', default_value='12345'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),

        include(os.path.join(LAUNCH, 'hardware.launch.py'),
                {'site': LaunchConfiguration('site'), 'payload': LaunchConfiguration('payload'),
                 'imu': LaunchConfiguration('imu')}),
        include(os.path.join(LAUNCH, 'dashboard.launch.py'), {'usb_camera': LaunchConfiguration('camera')}),

        ExecuteProcess(cmd=['python3', LaunchConfiguration('rtk_bridge'),
                            '--tcp-port', LaunchConfiguration('nmea_port')],
                       name='rtk_bridge', output='screen', condition=IfCondition(LaunchConfiguration('gnss'))),
        # The bridge needs a moment to find the receiver and open its port before the driver connects
        TimerAction(period=5.0, condition=IfCondition(LaunchConfiguration('gnss')), actions=[
            Node(package='nmea_navsat_driver', executable='nmea_tcpclient_driver', name='gnss', output='screen',
                 parameters=[{'ip': '127.0.0.1', 'port': LaunchConfiguration('nmea_port')}],
                 remappings=[('fix', 'gps/fix'), ('vel', 'gps/vel')])]),

        # Slamtec's own S2 launch, resolved only when the lidar is enabled
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_s2_launch.py'])),
            condition=IfCondition(LaunchConfiguration('lidar')),
            launch_arguments={'serial_port': LaunchConfiguration('lidar_port'),
                              'frame_id': 'lidar_link'}.items()),
    ])
