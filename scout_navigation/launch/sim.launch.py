"""Whole simulation in one launch: Gazebo, navigator and dashboard on simulated time."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PACKAGE_SHARE = get_package_share_directory('scout_navigation')
LAUNCH = os.path.join(PACKAGE_SHARE, 'launch')

FORWARDED = {'site': os.path.join(PACKAGE_SHARE, 'config', 'survey.yaml'), 'obstacles': 'true',
             'bystander': 'false', 'carts': 'false', 'gz_args': '-r -v 2',
             'world': 'flat_field.sdf.xacro', 'show_path': 'true'}


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in FORWARDED.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LAUNCH, 'gazebo.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in FORWARDED}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LAUNCH, 'dashboard.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ])
