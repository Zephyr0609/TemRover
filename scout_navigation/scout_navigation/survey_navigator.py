"""The navigator node: turns GNSS, odometry, IMU and LiDAR into /cmd_vel along the survey lines.

Subscribes: gps/fix, gps/vel, imu/data, odom, scan, emergency_stop, survey_mode.
Publishes: cmd_vel, estimated_pose, survey_status (JSON for the dashboard), survey_path, survey_lines,
driven_route, obstacle_points, and the map -> base_link transform.

A mission is a list of steps: ('drive', start, end) along a survey line, ('headland', start, end) on a
turn or the run past a line end, and ('turn', angle) for a spot turn. control_step() runs at 20 Hz.
"""
import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header, String
from tf2_ros import TransformBroadcaster

from .detour import lateral_offset, nearest_group, plan_detour
from .geodesy import LocalTangentPlane, wrap_to_pi
from .obstacle_monitor import line_frame, scan_points, to_world
from .pose_estimator import PoseEstimator
from .survey_grid import generate_survey_mission, survey_lines

# Every ROS parameter with its default; config/survey.yaml and the site files override these
PARAMETERS = [
    # Survey grid: GNSS position of line 1's start, direction of line 1 (rad from east), size (m)
    ('origin_latitude', -37.7963), ('origin_longitude', 144.9614), ('grid_bearing', 0.0),
    ('grid_width', 50.0), ('grid_length', 50.0), ('line_spacing', 5.0),
    # Driving: speed (m/s), turn rate limit (rad/s), braking (m/s^2), pure-pursuit lookahead (m)
    ('cruise_speed', 1.5), ('max_angular_velocity', 1.5),
    ('max_deceleration', 6.9), ('heading_gain', 1.5), ('lookahead_distance', 1.5),
    # When a run counts as finished, when the rover counts as stopped, and when it counts as lost
    ('arrival_tolerance', 0.05), ('halt_speed', 0.05), ('path_deviation_limit', 3.0),
    # Control loop period (s), wait after launch (s), uncertainty of the starting heading (rad)
    ('control_period', 0.05), ('startup_delay', 5.0), ('initial_heading_noise', 0.35),
    # Spot turns: rate limit, damping on the yaw rate, and the tolerances that end the turn
    ('rotation_rate', 1.0), ('rotation_damping', 0.6), ('rotation_tolerance', 0.04),
    ('rotation_settled_rate', 0.05),
    # Obstacles: stop distance, margins around an object, and extra margin before a hold releases
    ('obstacle_standoff', 0.5), ('obstacle_hysteresis', 0.1), ('obstacle_clearance', 0.15),
    ('hold_hysteresis', 0.5),
    # Rover and LiDAR geometry (m, rad); lidar_yaw is pi when the LiDAR is mounted facing backwards
    ('rover_half_width', 0.35), ('lidar_offset', 0.34),
    ('lidar_height', 0.395), ('lidar_yaw', 0.0), ('detection_field_of_view', 1.047), ('obstacle_range', 10.0),
    # Telling obstacles from ground: minimum height, range jump between objects, widest object angle
    ('minimum_obstacle_height', 0.15),
    ('range_step_threshold', 0.3), ('maximum_object_angle', 1.57),
    # Detours: how far ahead to plan, sideways acceleration allowed, length of each path segment
    ('detour_lookahead', 10.0), ('detour_lateral_acceleration', 1.0),
    ('detour_segment_length', 1.0),
    # Pose estimator (EKF) noise settings
    ('position_process_noise', 0.01), ('yaw_process_noise', 0.01),
    ('gnss_noise', 0.02), ('course_noise', 0.2), ('minimum_course_speed', 0.3),
    # gnss_offset: antenna position ahead of the rover centre (m); dead_reckoning: no IMU, use chassis yaw rate
    ('gnss_offset', -0.30), ('dead_reckoning', False), ('anchor_to_origin', False),
    # towed_length > 0 means the Tx and Rx carts are attached; start_mode: idle waits for Start on the dashboard
    ('towed_length', 0.0), ('start_mode', 'autonomous'), ('wait_at_line_start', False),
    # turn_radius 0 = spot turns, > 0 = driven U-turns; headland = extra run past each line end (m)
    ('turn_radius', 0.0), ('headland', 0.0), ('turn_segment_length', 0.5),
    # Driven route for the dashboard: spacing between stored points (m) and publish period (s)
    ('route_spacing', 0.25), ('route_period', 2.0),
]

# States in which the rover must stand still
HALTED_STATES = ('WAITING_FOR_FIX', 'STOPPED', 'COMPLETE', 'LOST', 'HOLDING')


def roll_pitch(orientation):
    """Roll and pitch from a quaternion, ignoring yaw."""
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return np.array([roll, pitch])


def path_message(points):
    """A map-frame Path through the given points."""
    message = Path()
    message.header.frame_id = 'map'
    for point in points:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


class SurveyNavigator(Node):
    """The only node that publishes /cmd_vel: follows the mission step by step."""

    def __init__(self):
        super().__init__('survey_navigator')
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)
        self.settings = {name: self.get_parameter(name).value for name, _ in PARAMETERS}

        self.tangent_plane = LocalTangentPlane(self.settings['origin_latitude'],
                                               self.settings['origin_longitude'])
        self.mission = []
        self.estimator = PoseEstimator(self.settings['position_process_noise'],
                                       self.settings['yaw_process_noise'], self.settings['gnss_noise'],
                                       self.settings['course_noise'],
                                       self.settings['minimum_course_speed'],
                                       self.settings['gnss_offset'], self.settings['initial_heading_noise'])

        # Index of the mission step being driven
        self.step_index = 0
        # The line whose start the rover last stopped at for the operator
        self.waited_line = 1
        self.commanded_speed = 0.0
        # Speed and yaw rate as measured by the chassis or the IMU
        self.measured_speed = 0.0
        self.yaw_rate = 0.0
        # Roll and pitch from the IMU, used to tell ground returns from obstacles
        self.attitude = np.zeros(2)
        # Obstacle points from the latest scan, in map coordinates
        self.obstacle_points = np.empty((0, 2))
        # Start and end of the survey line being driven, kept while a detour replaces its steps
        self.survey_line = None
        # Sideways shift of the current detour; its sign is the side already chosen
        self.detour_offset = 0.0
        self.emergency_stop = False
        self.has_fix = False
        # state: what the rover is doing; mode: what the operator allows (idle, autonomous, manual)
        self.state = 'WAITING_FOR_FIX'
        self.mode = self.settings['start_mode']
        # Heading a spot turn is aiming for
        self.turn_target = None
        self.started_at = None

        # Outputs: drive command, pose, dashboard status, obstacles, and latched paths
        self.command_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'estimated_pose', 10)
        self.status_publisher = self.create_publisher(String, 'survey_status', 10)
        self.obstacle_publisher = self.create_publisher(PointCloud2, 'obstacle_points', 10)
        self.route_publisher = self.create_publisher(
            Path, 'driven_route', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.route = []
        self.create_timer(self.settings['route_period'], self.publish_route)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.path_publisher = self.create_publisher(
            Path, 'survey_path', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.lines_publisher = self.create_publisher(
            Path, 'survey_lines', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Inputs: GNSS position and velocity, IMU, chassis odometry, LiDAR, and the dashboard buttons
        self.create_subscription(NavSatFix, 'gps/fix', self.on_fix, 10)
        # East and north antenna velocity from the receiver's RMC/VTG course and speed
        self.create_subscription(TwistStamped, 'gps/vel', self.on_velocity, 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        # dead_reckoning: no IMU, so the chassis feedback supplies the yaw rate; GNSS is still required
        self.create_subscription(Odometry, 'odom', self.on_chassis_odometry
                                 if self.settings['dead_reckoning'] else self.on_odometry, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(Bool, 'emergency_stop', self.on_emergency_stop, 10)
        self.create_subscription(String, 'survey_mode', self.on_mode, 10)
        self.create_timer(self.settings['control_period'], self.control_step)

        self.get_logger().info('Waiting for a GNSS fix; the rover must face along line 1')

    def publish_pose(self):
        """Publishes the estimated pose and the same pose as the map -> base_link transform."""
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x, message.pose.position.y = map(float, self.estimator.position)
        message.pose.orientation.z = float(np.sin(self.estimator.heading / 2.0))
        message.pose.orientation.w = float(np.cos(self.estimator.heading / 2.0))
        self.pose_publisher.publish(message)

        transform = TransformStamped()
        transform.header, transform.child_frame_id = message.header, 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = map(float, self.estimator.position)
        transform.transform.rotation = message.pose.orientation
        self.transform_broadcaster.sendTransform(transform)

    def publish_route(self):
        """The whole driven route, latched, so a page opened mid-survey still shows it."""
        message = path_message(self.route)
        message.header.stamp = self.get_clock().now().to_msg()
        self.route_publisher.publish(message)

    def record_route(self):
        """Stores the position once the rover has moved route_spacing since the last stored point."""
        position = self.estimator.position
        if not self.route or np.hypot(*(position - self.route[-1])) > self.settings['route_spacing']:
            self.route.append(tuple(map(float, position)))

    def publish_status(self):
        """One JSON line per control step for the operator dashboard."""
        cross_track = 0.0
        if self.survey_line is not None:
            cross_track = line_frame(self.estimator.position[None, :], *self.survey_line)[1][0]
        self.status_publisher.publish(String(data=json.dumps({
            'state': self.state, 'mode': self.mode, 'emergency_stop': self.emergency_stop,
            'x': float(self.estimator.position[0]), 'y': float(self.estimator.position[1]),
            'yaw_rate': float(self.yaw_rate), 'fix': self.has_fix,
            'extent': [self.settings['grid_width'], self.settings['grid_length']],
            'corridor': self.settings['rover_half_width'] + self.settings['obstacle_clearance'],
            'towed_length': self.settings['towed_length'],
            'lidar_yaw': self.settings['lidar_yaw'],
            'goal': self.goal(),
            'line': self.current_line(),
            'waiting_at_line_start': self.mode == 'idle' and self.waited_line == self.current_line() > 1,
            'lines': len(self.line_starts()),
            'speed': self.commanded_speed, 'heading': float(np.degrees(self.estimator.heading)),
            'cross_track': float(cross_track), 'detour': float(self.detour_offset),
            'obstacles': int(len(self.obstacle_points)),
            'blocked_in': float(min(self.blocking_distance(), 99.0)) if self.survey_line is not None else 99.0,
        })))

    def current_line(self):
        """Number of the survey line being driven, counting from 1."""
        return max(1, sum(start <= self.step_index for start in self.line_starts()))

    def grid_survey_lines(self):
        """Start and end of every survey line as surveyed, in map coordinates."""
        return survey_lines(self.settings['grid_width'], self.settings['grid_length'],
                            self.settings['line_spacing'], *self.grid)

    def line_start_ahead(self):
        """Distance to the start of this survey line while the operator still has to start it there, else None.

        Line 1 waits for Start before the rover moves at all; every later line waits at its start after the turn.
        """
        line = self.current_line()
        if not self.settings['wait_at_line_start'] or self.state != 'SURVEYING' or line == self.waited_line:
            return None
        start, end = self.grid_survey_lines()[line - 1]
        return -float(line_frame(self.estimator.position[None, :], start, end)[0][0])

    def line_starts(self):
        """Index of the first step of every survey line; detours splice extra steps into a line."""
        return [index for index, step in enumerate(self.mission)
                if step[0] == 'drive' and (index == 0 or self.mission[index - 1][0] != 'drive')]

    def mission_for(self, bearing, origin):
        """Builds the mission for a grid at the given bearing and origin."""
        return generate_survey_mission(
            self.settings['grid_width'], self.settings['grid_length'], self.settings['line_spacing'],
            bearing, origin, self.settings['turn_radius'], self.settings['headland'],
            self.settings['turn_segment_length'])

    def goal(self):
        """End point of the current survey line step, for the dashboard; None on a turn."""
        if self.step_index < len(self.mission) and self.mission[self.step_index][0] == 'drive':
            return [round(float(value), 2) for value in self.mission[self.step_index][2]]
        return None

    def publish_path(self):
        """Publishes the route as generated (detours never drawn) and the survey lines' real ends."""
        route = [point for step in self.mission if step[0] != 'turn' for point in step[1:]]
        self.path_publisher.publish(path_message(route))
        self.lines_publisher.publish(path_message([point for line in self.grid_survey_lines() for point in line]))

    def on_fix(self, message):
        """GNSS position: the first fix lays out the survey, every fix corrects the estimator."""
        position = self.tangent_plane.to_local(message.latitude, message.longitude)
        if not self.has_fix:
            self.start_survey(position)
        self.estimator.update_position(position)

    def start_survey(self, antenna_position):
        """No alignment run: the rover stands facing along line 1, and GNSS course corrects that once it moves."""
        bearing = self.settings['grid_bearing']
        self.estimator.state[2] = bearing
        self.estimator.state[:2] = self.estimator.antenna_to_base(antenna_position)
        if self.settings['anchor_to_origin']:
            origin = np.zeros(2)
        else:
            # Line 1's run-in starts where the rover stands, so the grid begins one headland ahead
            origin = self.estimator.position + self.settings['headland'] * np.array([np.cos(bearing), np.sin(bearing)])
        self.grid = (bearing, origin)
        self.mission = self.mission_for(bearing, origin)
        self.publish_path()
        self.has_fix = True
        self.get_logger().info(f'Survey laid out at {np.degrees(bearing):+.1f} deg, {len(self.mission)} mission steps')

    def on_velocity(self, message):
        """GNSS velocity: its direction of travel corrects the estimated heading."""
        self.estimator.update_course(np.array([message.twist.linear.x, message.twist.linear.y]), self.yaw_rate)

    def on_odometry(self, message):
        """Chassis speed, used to predict the motion between GNSS fixes."""
        self.measured_speed = message.twist.twist.linear.x

    def on_chassis_odometry(self, message):
        """Without an IMU the chassis feedback is the only yaw rate the estimator can integrate."""
        self.measured_speed = message.twist.twist.linear.x
        self.yaw_rate = message.twist.twist.angular.z

    def on_imu(self, message):
        """IMU: yaw rate for the estimator, roll and pitch for the LiDAR ground filter."""
        self.yaw_rate = message.angular_velocity.z
        self.attitude = roll_pitch(message.orientation)

    def on_scan(self, message):
        """LiDAR scan: keeps the returns that are obstacles ahead and places them on the map."""
        points = scan_points(message, self.attitude, self.settings['lidar_height'],
                             self.settings['minimum_obstacle_height'],
                             self.settings['range_step_threshold'],
                             self.settings['maximum_object_angle'], self.settings['lidar_yaw'],
                             self.settings['detection_field_of_view'], self.settings['obstacle_range'])
        self.obstacle_points = to_world(points, self.estimator.position, self.estimator.heading,
                                        self.settings['lidar_offset'])
        header = Header(stamp=message.header.stamp, frame_id='map')
        self.obstacle_publisher.publish(create_cloud_xyz32(
            header, [(float(x), float(y), 0.0) for x, y in self.obstacle_points]))

    def on_emergency_stop(self, message):
        """Dashboard emergency stop; releasing it restarts the survey from line 1."""
        if self.emergency_stop and not message.data:
            self.restart()
        self.emergency_stop = message.data

    def restart(self):
        """Clearing an emergency stop restarts the survey from line 1 on the same grid, in idle until Start."""
        self.step_index = 0
        self.waited_line = 1
        self.survey_line = None
        self.detour_offset = 0.0
        self.turn_target = None
        self.obstacle_points = np.empty((0, 2))
        self.route = []
        if self.mission:
            self.mission = self.mission_for(*self.grid)
            self.publish_path()
        self.publish_route()
        self.mode = 'idle'
        self.halt()
        self.get_logger().info('emergency stop cleared: survey reset to line 1, waiting for Start')

    def on_mode(self, message):
        """idle holds the rover, manual hands cmd_vel to the operator, autonomous runs the survey."""
        self.mode = message.data
        self.get_logger().info(f'mode -> {self.mode}')
        # Leaving autonomy must not leave the last drive command running
        self.halt()

    def drive_frame(self):
        """Unit direction, distance still to run, and signed offset from the current straight run."""
        _, start, end = self.mission[self.step_index]
        offset = end - start
        length = float(np.hypot(*offset))
        direction = offset / length
        travel = self.estimator.position - start
        normal = np.array([-direction[1], direction[0]])
        return direction, length - float(travel @ direction), float(travel @ normal)

    def heading_error(self, target):
        """Signed angle from the estimated heading to the target heading."""
        return wrap_to_pi(target - self.estimator.heading)

    def rotation_settled(self, target):
        """True when a spot turn has reached its heading and stopped rotating."""
        return (abs(self.heading_error(target)) < self.settings['rotation_tolerance']
                and abs(self.yaw_rate) < self.settings['rotation_settled_rate'])

    def braking_distance(self):
        """Distance needed to stop from the commanded speed."""
        return self.commanded_speed ** 2 / (2.0 * self.settings['max_deceleration'])

    def next_state(self):
        """Decides the state for this control step from the mission, the obstacles and the pose."""
        if self.emergency_stop:
            return 'STOPPED'
        settling = (self.get_clock().now() - self.started_at).nanoseconds * 1e-9
        if not self.has_fix or settling < self.settings['startup_delay']:
            return 'WAITING_FOR_FIX'
        if self.step_index >= len(self.mission):
            return 'COMPLETE'

        if self.mission[self.step_index][0] == 'turn':
            return 'TURNING'

        hold_distance = self.settings['obstacle_standoff'] + self.braking_distance()
        if self.state == 'HOLDING':
            hold_distance += self.settings['hold_hysteresis']
        if self.blocking_distance() < hold_distance:
            return 'HOLDING'
        if abs(self.drive_frame()[2]) > self.settings['path_deviation_limit']:
            return 'LOST'
        return 'SURVEYING' if self.mission[self.step_index][0] == 'drive' else 'HEADLAND'

    def run_end(self):
        """Index one past the last step of the current run: a survey line or a headland turn."""
        kind = self.mission[self.step_index][0]
        index = self.step_index
        while index < len(self.mission) and self.mission[index][0] == kind:
            index += 1
        return index

    def steps_length(self, first, last):
        """Total length of the mission steps first..last-1."""
        return sum(float(np.hypot(*(end - start))) for _, start, end in self.mission[first:last])

    def run_remaining(self):
        """Distance left in this run; along the original line while a detour is spliced into it."""
        if self.mission[self.step_index][0] == 'drive':
            start, end = self.survey_line
            along = line_frame(self.estimator.position[None, :], start, end)[0][0]
            return float(np.hypot(*(end - start))) - along
        return self.drive_frame()[1] + self.steps_length(self.step_index + 1, self.run_end())

    def stop_ahead(self):
        """Index of the next spot turn, or the mission end: the only places the rover must stop."""
        index = self.run_end()
        while index < len(self.mission) and self.mission[index][0] != 'turn':
            index += 1
        return index

    def blocking_distance(self):
        """Distance along the planned drive steps to the first return inside the swept corridor."""
        corridor = self.settings['rover_half_width'] + self.settings['obstacle_clearance']
        travelled = 0.0
        for index in range(self.step_index, self.run_end()):
            _, start, end = self.mission[index]
            length = float(np.hypot(*(end - start)))
            along, across = line_frame(self.obstacle_points, start, end)
            entry = line_frame(self.estimator.position[None, :], start, end)[0][0] if index == self.step_index else 0.0
            inside = (along > entry) & (along < length) & (np.abs(across) < corridor)
            if inside.any():
                return travelled + float(along[inside].min()) - entry
            travelled += length - entry
        return np.inf

    def avoid_obstacles(self):
        """Splices a detour into the run when the corridor ahead is blocked within lookahead."""
        if not self.blocking_distance() < self.settings['detour_lookahead']:
            return

        start, end = self.survey_line
        along, across = line_frame(self.obstacle_points, start, end)
        s_now, d_now = (value[0] for value in line_frame(self.estimator.position[None, :], start, end))
        planned = np.array([point for step in self.mission[self.step_index:self.run_end()]
                            for point in step[1:]])
        planned_across = line_frame(planned, start, end)[1]
        corridor = self.settings['rover_half_width'] + self.settings['obstacle_clearance']
        lane = ((along > s_now) & (along < s_now + self.settings['detour_lookahead'])
                & (across > planned_across.min() - corridor)
                & (across < planned_across.max() + corridor))
        margin = self.settings['obstacle_clearance'] + self.settings['obstacle_hysteresis']
        if not lane.any():
            return
        # Plan around the nearest object only; a second one further on gets its own plan later
        group_along, group_across = nearest_group(along[lane], across[lane], 2.0 * margin)
        room = group_along.min() - margin - s_now
        committed = np.sign(self.detour_offset)
        offset = lateral_offset(group_across.min(), group_across.max(),
                                self.settings['rover_half_width'], margin, committed)
        if abs(offset) < self.settings['arrival_tolerance']:
            return
        # No room for a ramp: keep the lateral position and only extend the hold along the line
        if room < self.settings['detour_segment_length']:
            offset = d_now
        slope_now = -np.tan(self.heading_error(np.arctan2(*(end - start)[::-1])))
        self.mission[self.step_index:self.run_end()] = plan_detour(
            start, end, s_now, d_now, slope_now, (group_along.min(), group_along.max()), offset,
            margin, self.settings['cruise_speed'], self.settings['detour_lateral_acceleration'],
            self.settings['detour_segment_length'])
        self.detour_offset = offset
        self.get_logger().info(f'detour {offset:+.2f} m around {len(group_along)} returns '
                               f'{group_along.min() - s_now:.1f} m ahead, extent '
                               f'{group_across.min():+.2f}..{group_across.max():+.2f}, committed {committed:+.0f}')

    def rotate_command(self, target):
        """Rate damping is needed — proportional control alone limit-cycles on this mass."""
        effort = (self.settings['heading_gain'] * self.heading_error(target)
                  - self.settings['rotation_damping'] * self.yaw_rate)
        limit = self.settings['rotation_rate']

        command = Twist()
        command.angular.z = float(np.clip(effort, -limit, limit))
        return command

    def lookahead_point(self):
        """The point lookahead_distance further along the planned path than the rover's foot on it.

        The path runs on through lines, detours and U-turns up to the next spot turn; past its end it
        carries straight on, so the point is always a full lookahead away.
        """
        _, start, end = self.mission[self.step_index]
        direction = (end - start) / np.hypot(*(end - start))
        foot = start + direction * np.clip((self.estimator.position - start) @ direction, 0.0, np.hypot(*(end - start)))
        reach, index = self.settings['lookahead_distance'], self.step_index
        while True:
            _, start, end = self.mission[index]
            to_end = float(np.hypot(*(end - foot)))
            following = index + 1
            if reach <= to_end or following >= len(self.mission) or self.mission[following][0] == 'turn':
                return foot + reach * (end - start) / np.hypot(*(end - start))
            reach, foot, index = reach - to_end, end, following

    def drive_command(self, target, remaining):
        """Pure pursuit: the arc through the lookahead point, so turns start before the rover reaches them."""
        self.commanded_speed = min(self.settings['cruise_speed'], float(np.sqrt(
            2.0 * self.settings['max_deceleration'] * remaining)))
        offset = target - self.estimator.position
        bearing_error = self.heading_error(np.arctan2(offset[1], offset[0]))
        curvature = 2.0 * np.sin(bearing_error) / np.hypot(*offset)
        limit = self.settings['max_angular_velocity']

        command = Twist()
        command.linear.x = self.commanded_speed
        command.angular.z = float(np.clip(self.commanded_speed * curvature, -limit, limit))
        return command

    def halt(self):
        """Commands zero speed."""
        self.commanded_speed = 0.0
        self.command_publisher.publish(Twist())

    def control_step(self):
        """One 20 Hz cycle: predict the pose, replan around obstacles, pick the state, drive."""
        if self.started_at is None:
            self.started_at = self.get_clock().now()

        self.estimator.predict(self.measured_speed, self.yaw_rate, self.settings['control_period'])
        self.publish_pose()
        self.publish_status()
        if self.has_fix:
            self.record_route()
        # manual: another node drives; idle: paused by the operator, progress is kept
        if self.mode == 'manual':
            return
        if self.mode == 'idle':
            self.halt()
            return

        # Replan first, then decide: a hold must only fire if the freshest plan is still blocked
        if self.step_index < len(self.mission) and self.mission[self.step_index][0] == 'drive':
            if self.survey_line is None:
                self.survey_line = self.mission[self.step_index][1:]
            self.avoid_obstacles()

        state = self.next_state()
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state} (step {self.step_index})')
            self.state = state

        if state in HALTED_STATES:
            self.halt()
            return

        if state == 'TURNING':
            self.commanded_speed = 0.0
            if self.turn_target is None:
                self.turn_target = wrap_to_pi(self.estimator.heading
                                              + self.mission[self.step_index][1])
            if self.rotation_settled(self.turn_target):
                self.turn_target = None
                self.survey_line = None
                self.detour_offset = 0.0
                self.step_index += 1
                self.halt()
                return
            self.command_publisher.publish(self.rotate_command(self.turn_target))
            return

        remaining = self.run_remaining()
        # Stop at the start of each new survey line until the operator presses Start
        line_start = self.line_start_ahead()
        if line_start is not None and line_start <= self.settings['arrival_tolerance']:
            self.waited_line = self.current_line()
            self.mode = 'idle'
            self.halt()
            self.get_logger().info(f'at the start of line {self.waited_line}: waiting for Start')
            return
        if remaining > self.settings['arrival_tolerance']:
            # Move on to the next short step of this run once the current one is passed
            if self.drive_frame()[1] <= 0.0 and self.step_index + 1 < self.run_end():
                self.step_index += 1
            to_stop = remaining + self.steps_length(self.run_end(), self.stop_ahead())
            if line_start is not None:
                to_stop = min(to_stop, line_start)
            self.command_publisher.publish(self.drive_command(self.lookahead_point(), to_stop))
            return

        # A line flows straight into its headland turn and back; only a spot turn needs a standstill
        if self.run_end() < self.stop_ahead():
            self.step_index = self.run_end()
            self.survey_line = None
            self.detour_offset = 0.0
            return

        # Stop fully before an in-place turn; a rolling rover slides straight through it
        self.halt()
        if abs(self.measured_speed) < self.settings['halt_speed']:
            self.step_index = self.run_end()
            self.survey_line = None
            self.detour_offset = 0.0


def main(args=None):
    rclpy.init(args=args)
    navigator = SurveyNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
