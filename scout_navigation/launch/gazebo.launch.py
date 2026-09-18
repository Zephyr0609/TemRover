import os
import subprocess
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

BRIDGED_TOPICS = [
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
    '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
    '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
    '/world/survey_field/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
    '/survey_camera@sensor_msgs/msg/Image[ignition.msgs.Image',
    '/front_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
]

PACKAGE_SHARE = get_package_share_directory('scout_navigation')
SURVEY = os.path.join(PACKAGE_SHARE, 'config', 'survey.yaml')
PAYLOAD = os.path.join(PACKAGE_SHARE, 'config', 'payload.yaml')
OBSTACLES = os.path.join(PACKAGE_SHARE, 'config', 'obstacles.yaml')


def spawn_obstacles():
    """One spawn node per obstacle in the layout, so the test set lives in config not in code."""
    layout = yaml.safe_load(open(OBSTACLES))['obstacles']
    return [Node(package='ros_gz_sim', executable='create', output='screen',
                 condition=IfCondition(LaunchConfiguration('obstacles')),
                 arguments=['-file', os.path.join(PACKAGE_SHARE, 'worlds', item['model']),
                            '-name', item['name'],
                            '-x', str(item['pose'][0]), '-y', str(item['pose'][1]),
                            '-z', str(item['pose'][2])])
            for item in layout]


def render_world(context):
    """Renders the world with the site's GPS origin so simulated fixes land where the config expects."""
    site = LaunchConfiguration('site').perform(context)
    origin = yaml.safe_load(open(site))['/**']['ros__parameters']
    template = os.path.join(PACKAGE_SHARE, 'worlds', LaunchConfiguration('world').perform(context))
    rendered = subprocess.run(
        ['xacro', template, f"latitude:={origin['origin_latitude']}",
         f"longitude:={origin['origin_longitude']}"], check=True, capture_output=True, text=True)

    world_file = tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False)
    world_file.write(rendered.stdout)
    world_file.close()
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world_file.name, ' ', LaunchConfiguration('gz_args')]}.items())]


def generate_launch_description():
    description = os.path.join(get_package_share_directory('scout_description'), 'urdf',
                               'scout_v2.xacro')
    extras = os.path.join(PACKAGE_SHARE, 'description', 'temrover_extras.xacro')
    survey_path = os.path.join(PACKAGE_SHARE, 'worlds', 'survey_path.sdf')

    robot_description = ParameterValue(
        Command(['xacro ', description, ' urdf_extras:=', extras,
                 ' carts:=', LaunchConfiguration('carts')]), value_type=str)
    # The towed train is only in the navigator's parameters when the carts are actually attached
    payload = PythonExpression(["'", PAYLOAD, "' if '", LaunchConfiguration('carts'),
                                "' == 'true' else '", SURVEY, "'"])

    return LaunchDescription([
        DeclareLaunchArgument('site', default_value=SURVEY,
                              description='Site parameters layered over survey.yaml'),

        DeclareLaunchArgument('show_path', default_value='true',
                              description='Draw the planned survey path into the scene'),

        DeclareLaunchArgument('obstacles', default_value='true',
                              description='Place the test obstacles on the survey lines'),

        DeclareLaunchArgument('spawn_height', default_value='0.4',
                              description='Spawn height; raise it for the rough world'),

        DeclareLaunchArgument('carts', default_value='true',
                              description='Tow the Tx and Rx carts behind the rover'),

        DeclareLaunchArgument('world', default_value='flat_field.sdf.xacro',
                              description='World template: flat_field.sdf.xacro or rough_field.sdf.xacro'),

        DeclareLaunchArgument('gz_args', default_value='-r -v 2',
                              description='Extra Gazebo arguments, add -s to run headless'),

        OpaqueFunction(function=render_world),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description,
                                           'use_sim_time': True}]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'temrover',
                        '-x', '0', '-y', '0', '-z', LaunchConfiguration('spawn_height')]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             condition=IfCondition(LaunchConfiguration('show_path')),
             arguments=['-file', survey_path, '-name', 'survey_path',
                        '-x', '0', '-y', '0', '-z', '0']),

        *spawn_obstacles(),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=BRIDGED_TOPICS + ['--ros-args', '-r',
                       '/world/survey_field/dynamic_pose/info:=/link_poses'],
             parameters=[{'use_sim_time': True}]),

        Node(package='scout_navigation', executable='survey_navigator', name='survey_navigator',
             output='screen', emulate_tty=True,
             parameters=[SURVEY, payload, LaunchConfiguration('site'), {'use_sim_time': True}]),
    ])
