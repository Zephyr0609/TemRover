import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    survey = os.path.join(get_package_share_directory('scout_navigation'), 'config', 'survey.yaml')

    return LaunchDescription([
        Node(package='scout_navigation', executable='survey_navigator', name='survey_navigator',
             output='screen', parameters=[survey]),

        Node(package='scout_navigation', executable='scout_can_bridge', name='scout_can_bridge',
             output='screen', parameters=[survey]),
    ])
