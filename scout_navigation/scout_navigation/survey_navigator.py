import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistWithCovarianceStamped
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
from .survey_grid import generate_survey_mission

PARAMETERS = [
    ('origin_latitude', -37.7963), ('origin_longitude', 144.9614), ('grid_bearing', 0.0),
    ('grid_width', 50.0), ('grid_length', 50.0), ('line_spacing', 5.0), ('point_spacing', 0.1),
    ('turn_radius', 0.0), ('cruise_speed', 1.5), ('max_angular_velocity', 1.5),
    ('max_deceleration', 6.9), ('heading_gain', 1.5), ('cross_track_gain', 0.8),
    ('arrival_tolerance', 0.05), ('halt_speed', 0.05), ('path_deviation_limit', 3.0),
    ('control_period', 0.05), ('startup_delay', 5.0), ('alignment_distance', 8.0), ('alignment_speed', 0.3),
    ('rotation_rate', 1.0), ('rotation_damping', 0.6), ('rotation_tolerance', 0.04),
    ('rotation_settled_rate', 0.05),
    ('obstacle_standoff', 0.5), ('obstacle_hysteresis', 0.1), ('obstacle_clearance', 0.15),
    ('hold_hysteresis', 0.5),
    ('rover_half_width', 0.35), ('lidar_offset', 0.34),
    ('lidar_height', 0.395), ('lidar_yaw', 0.0), ('detection_field_of_view', 1.047),
    ('minimum_obstacle_height', 0.15),
    ('range_step_threshold', 0.3), ('maximum_object_angle', 1.57),
    ('detour_lookahead', 10.0), ('detour_lateral_acceleration', 1.0),
    ('detour_segment_length', 1.0),
    ('position_process_noise', 0.01), ('yaw_process_noise', 0.01),
    ('gnss_noise', 0.02), ('course_noise', 0.2), ('minimum_course_speed', 0.3),
    ('gnss_offset', -0.30), ('dead_reckoning', False), ('anchor_to_origin', False),
    ('towed_length', 0.0), ('start_mode', 'autonomous'),
    ('route_spacing', 0.25), ('route_period', 2.0),
]

HALTED_STATES = ('WAITING_FOR_FIX', 'STOPPED', 'COMPLETE', 'LOST', 'HOLDING')


def roll_pitch(orientation):
    """Roll and pitch from a quaternion, ignoring yaw."""
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return np.array([roll, pitch])


class SurveyNavigator(Node):
    """Sole owner of /cmd_vel — walks a mission of straight runs and in-place turns."""

    def __init__(self):
        super().__init__('survey_navigator')
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)
        self.settings = {name: self.get_parameter(name).value for name, _ in PARAMETERS}

        self.tangent_plane = LocalTangentPlane(self.settings['origin_latitude'],
                                               self.settings['origin_longitude'])
        self.mission = []
        self.alignment_track = []
        self.estimator = PoseEstimator(self.settings['position_process_noise'],
                                       self.settings['yaw_process_noise'], self.settings['gnss_noise'],
                                       self.settings['course_noise'],
                                       self.settings['minimum_course_speed'],
                                       self.settings['gnss_offset'])

        self.step_index = 0
        self.commanded_speed = 0.0
        self.measured_speed = 0.0
        self.yaw_rate = 0.0
        self.attitude = np.zeros(2)
        self.obstacle_points = np.empty((0, 2))
        self.survey_line = None
        self.detour_offset = 0.0
        self.emergency_stop = False
        self.has_fix = False
        self.state = 'WAITING_FOR_FIX'
        self.mode = self.settings['start_mode']
        self.turn_target = None
        self.started_at = None

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
        self.create_subscription(NavSatFix, 'gps/fix', self.on_fix, 10)
        self.create_subscription(TwistWithCovarianceStamped, 'gps/fix_velocity', self.on_velocity, 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        # dead_reckoning: no IMU, so the chassis feedback supplies the yaw rate; GNSS is still required
        self.create_subscription(Odometry, 'odom', self.on_chassis_odometry
                                 if self.settings['dead_reckoning'] else self.on_odometry, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(Bool, 'emergency_stop', self.on_emergency_stop, 10)
        self.create_subscription(String, 'survey_mode', self.on_mode, 10)
        self.create_timer(self.settings['control_period'], self.control_step)

        self.get_logger().info('Waiting for a fix, then a straight run to observe heading')

    def publish_pose(self):
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
        message = Path()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        for point in self.route:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = point
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.route_publisher.publish(message)

    def record_route(self):
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
            # Two spot turns separate consecutive lines
            'line': 1 + sum(step[0] == 'turn' for step in self.mission[:self.step_index]) // 2,
            'lines': 1 + sum(step[0] == 'turn' for step in self.mission) // 2,
            'speed': self.commanded_speed, 'heading': float(np.degrees(self.estimator.heading)),
            'cross_track': float(cross_track), 'detour': float(self.detour_offset),
            'obstacles': int(len(self.obstacle_points)),
            'blocked_in': float(min(self.blocking_distance(), 99.0)) if self.survey_line is not None else 99.0,
        })))

    def goal(self):
        if self.step_index < len(self.mission) and self.mission[self.step_index][0] == 'drive':
            return [round(float(value), 2) for value in self.mission[self.step_index][2]]
        return None

    def publish_path(self):
        """Publishes the survey lines as generated; detours are spliced into the mission but never drawn."""
        message = Path()
        message.header.frame_id = 'map'
        for step in self.mission:
            if step[0] != 'drive':
                continue
            for point in step[1:]:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
                pose.pose.orientation.w = 1.0
                message.poses.append(pose)
        self.path_publisher.publish(message)

    def align(self):
        """Drives straight to observe heading — a single antenna cannot find it standing still."""
        self.alignment_track.append(self.estimator.position.copy())
        track = np.array(self.alignment_track)
        if np.hypot(*(track[-1] - track[0])) < self.settings['alignment_distance']:
            # Heading is unknown yet, but the gyro can still hold the run straight against any drag
            command = Twist()
            command.linear.x = self.settings['alignment_speed']
            command.angular.z = -self.settings['rotation_damping'] * self.yaw_rate
            self.commanded_speed = command.linear.x
            self.command_publisher.publish(command)
            return

        direction = np.linalg.svd(track - track.mean(axis=0))[2][0]
        if direction @ (track[-1] - track[0]) < 0.0:
            direction = -direction

        self.estimator.state[2] = np.arctan2(direction[1], direction[0])
        if self.settings['anchor_to_origin']:
            bearing, origin = self.settings['grid_bearing'], np.zeros(2)
        else:
            bearing, origin = self.estimator.heading + self.settings['grid_bearing'], track[0]
        self.grid = (bearing, origin)
        self.mission = generate_survey_mission(
            self.settings['grid_width'], self.settings['grid_length'],
            self.settings['line_spacing'], bearing, origin)
        self.publish_path()
        self.get_logger().info(f'Aligned to {np.degrees(self.estimator.heading):+.1f} deg, '
                               f'{len(self.mission)} mission steps')
        self.halt()

    def on_fix(self, message):
        position = self.tangent_plane.to_local(message.latitude, message.longitude)
        if not self.has_fix:
            self.estimator.state[:2] = self.estimator.antenna_to_base(position)
            self.has_fix = True
        self.estimator.update_position(position)

    def on_velocity(self, message):
        self.estimator.update_course(np.array([message.twist.twist.linear.x,
                                               message.twist.twist.linear.y]), self.yaw_rate)

    def on_odometry(self, message):
        self.measured_speed = message.twist.twist.linear.x

    def on_chassis_odometry(self, message):
        """Without an IMU the chassis feedback is the only yaw rate the estimator can integrate."""
        self.measured_speed = message.twist.twist.linear.x
        self.yaw_rate = message.twist.twist.angular.z

    def on_imu(self, message):
        self.yaw_rate = message.angular_velocity.z
        self.attitude = roll_pitch(message.orientation)

    def on_scan(self, message):
        points = scan_points(message, self.attitude, self.settings['lidar_height'],
                             self.settings['minimum_obstacle_height'],
                             self.settings['range_step_threshold'],
                             self.settings['maximum_object_angle'], self.settings['lidar_yaw'],
                             self.settings['detection_field_of_view'])
        self.obstacle_points = to_world(points, self.estimator.position, self.estimator.heading,
                                        self.settings['lidar_offset'])
        header = Header(stamp=message.header.stamp, frame_id='map')
        self.obstacle_publisher.publish(create_cloud_xyz32(
            header, [(float(x), float(y), 0.0) for x, y in self.obstacle_points]))

    def on_emergency_stop(self, message):
        if self.emergency_stop and not message.data:
            self.restart()
        self.emergency_stop = message.data

    def restart(self):
        """Clearing an emergency stop restarts the survey from line 1 on the same grid, in idle until Start."""
        self.step_index = 0
        self.survey_line = None
        self.detour_offset = 0.0
        self.turn_target = None
        self.obstacle_points = np.empty((0, 2))
        self.route = []
        if self.mission:
            bearing, origin = self.grid
            self.mission = generate_survey_mission(
                self.settings['grid_width'], self.settings['grid_length'],
                self.settings['line_spacing'], bearing, origin)
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
        return wrap_to_pi(target - self.estimator.heading)

    def rotation_settled(self, target):
        return (abs(self.heading_error(target)) < self.settings['rotation_tolerance']
                and abs(self.yaw_rate) < self.settings['rotation_settled_rate'])

    def braking_distance(self):
        return self.commanded_speed ** 2 / (2.0 * self.settings['max_deceleration'])

    def next_state(self):
        if self.emergency_stop:
            return 'STOPPED'
        settling = (self.get_clock().now() - self.started_at).nanoseconds * 1e-9
        if not self.has_fix or settling < self.settings['startup_delay']:
            return 'WAITING_FOR_FIX'
        if not self.mission:
            return 'ALIGNING'
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
        return 'SURVEYING'

    def run_end(self):
        """Index one past the last drive step of the current straight run."""
        index = self.step_index
        while index < len(self.mission) and self.mission[index][0] == 'drive':
            index += 1
        return index

    def run_remaining(self):
        start, end = self.survey_line
        along = line_frame(self.estimator.position[None, :], start, end)[0][0]
        return float(np.hypot(*(end - start))) - along

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

    def drive_command(self, direction, remaining, cross_track):
        self.commanded_speed = min(self.settings['cruise_speed'], float(np.sqrt(
            2.0 * self.settings['max_deceleration'] * remaining)))
        heading_error = self.heading_error(np.arctan2(direction[1], direction[0]))
        limit = self.settings['max_angular_velocity']

        command = Twist()
        command.linear.x = self.commanded_speed
        command.angular.z = float(np.clip(
            self.settings['heading_gain'] * heading_error
            - self.settings['cross_track_gain'] * cross_track, -limit, limit))
        return command

    def halt(self):
        self.commanded_speed = 0.0
        self.command_publisher.publish(Twist())

    def control_step(self):
        if self.started_at is None:
            self.started_at = self.get_clock().now()

        self.estimator.predict(self.measured_speed, self.yaw_rate, self.settings['control_period'])
        self.publish_pose()
        self.publish_status()
        if self.has_fix:
            self.record_route()
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

        if state == 'ALIGNING':
            self.align()
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
        if remaining > self.settings['arrival_tolerance']:
            if self.drive_frame()[1] <= 0.0 and self.step_index + 1 < self.run_end():
                self.step_index += 1
            direction, _, cross_track = self.drive_frame()
            self.command_publisher.publish(self.drive_command(direction, remaining, cross_track))
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
