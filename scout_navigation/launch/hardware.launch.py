"""Real rover: navigator and CAN bridge, with the towed train and an optional site layered over survey.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG = os.path.join(get_package_share_directory('scout_navigation'), 'config')
SURVEY = os.path.join(CONFIG, 'survey.yaml')
PAYLOAD = os.path.join(CONFIG, 'payload.yaml')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value=SURVEY,
                              description='Site parameters layered over survey.yaml, e.g. south_lawn.yaml'),
        DeclareLaunchArgument('payload', default_value=PAYLOAD,
                              description='Towed-train parameters; pass survey.yaml when driving without the carts'),

        Node(package='scout_navigation', executable='survey_navigator', name='survey_navigator',
             output='screen', emulate_tty=True,
             parameters=[SURVEY, LaunchConfiguration('payload'), LaunchConfiguration('site')]),

        Node(package='scout_navigation', executable='scout_can_bridge', name='scout_can_bridge',
             output='screen', parameters=[SURVEY]),
    ])
