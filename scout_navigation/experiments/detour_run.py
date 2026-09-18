"""Records the rover against the test obstacles and reports clearance and steering smoothness."""
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from survey_config import load_obstacles
from tf2_msgs.msg import TFMessage

MODEL_NAME = 'temrover'
SWING_RADIUS = 0.58
LINE_SPACING = 5.0
LINE_LENGTH = 50.0

# A bystander steps away once the rover has waited beside it, proving a swing hold releases
BYSTANDER_TRIGGER = 3.0
BYSTANDER_DWELL = 3.0

LAYOUT = load_obstacles()
OBSTACLES = {item['name']: (item['pose'][0], item['pose'][1], *item['half_size'])
             for item in LAYOUT}
BYSTANDER = next((item for item in LAYOUT if item.get('bystander')), None)


def remove_request(name):
    return ['ign', 'service', '-s', '/world/survey_field/remove',
            '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000', '--req', f'name: "{name}" type: MODEL']



def surface_distance(position, obstacle):
    """Distance from a point to the outside of an axis-aligned box, or of a circle."""
    centre_x, centre_y, half_x, half_y = obstacle
    if half_x == half_y:
        return np.hypot(position[0] - centre_x, position[1] - centre_y) - half_x
    gap = np.maximum(np.abs(position - np.array([centre_x, centre_y])) - np.array([half_x, half_y]), 0.0)
    return np.hypot(*gap)


class DetourRecorder(Node):
    """Logs true rover position, commanded velocities and the latest planned path."""

    def __init__(self):
        super().__init__('detour_recorder',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.track = []
        self.commands = []
        self.estimates = []
        self.planned = []
        self.arrived_at_corner = None
        self.bystander_left = None
        self.create_subscription(TFMessage, 'link_poses', self.on_poses, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.create_subscription(PoseStamped, 'estimated_pose', self.on_estimate, 10)
        self.create_subscription(Path, 'survey_path', self.on_path, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_poses(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == MODEL_NAME:
                rotation = transform.transform.rotation
                self.track.append((self.seconds(), transform.transform.translation.x,
                                   transform.transform.translation.y,
                                   2.0 * np.arctan2(rotation.z, rotation.w)))
                self.watch_corner(np.array(self.track[-1][1:3]))

    def watch_corner(self, position):
        """Removes the bystander once the rover has waited beside it for the dwell time."""
        if BYSTANDER is None or self.bystander_left is not None:
            return
        if np.hypot(*(position - np.array(BYSTANDER['pose'][:2]))) > BYSTANDER_TRIGGER:
            self.arrived_at_corner = None
            return
        if self.arrived_at_corner is None:
            self.arrived_at_corner = self.seconds()
        if self.seconds() - self.arrived_at_corner > BYSTANDER_DWELL:
            subprocess.run(remove_request(BYSTANDER['name']), check=True, capture_output=True)
            self.bystander_left = self.seconds()
            print(f"bystander {BYSTANDER['name']} removed at {self.bystander_left:.1f} s sim time")

    def on_command(self, message):
        self.commands.append((self.seconds(), message.linear.x, message.angular.z))

    def on_estimate(self, message):
        rotation = message.pose.orientation
        self.estimates.append((self.seconds(), message.pose.position.x, message.pose.position.y,
                               2.0 * np.arctan2(rotation.z, rotation.w)))

    def on_path(self, message):
        self.planned = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]


def heading_error(track, estimates):
    """Estimated minus true heading, pairing each estimate with the nearest truth sample in time."""
    nearest = np.searchsorted(track[:, 0], estimates[:, 0]).clip(0, len(track) - 1)
    error = estimates[:, 3] - track[nearest, 3]
    return estimates[:, 0], np.degrees(np.arctan2(np.sin(error), np.cos(error)))


def report(recorder, filename):
    track = np.array(recorder.track)
    commands = np.array(recorder.commands)
    start = track[0, 0]
    figure, axes = plt.subplots(1, 4, figsize=(22, 5))

    if recorder.estimates:
        times, error = heading_error(track, np.array(recorder.estimates))
        axes[3].plot(times - start, error, lw=0.7, color='C4')
        print(f'heading estimate error: rms {np.sqrt(np.mean(error ** 2)):.2f} deg  '
              f'max {np.abs(error).max():.2f} deg')
    axes[3].set(xlabel='time [s]', ylabel='estimated minus true heading [deg]',
                title='EKF heading error')

    for line in range(3):
        axes[0].axhline(line * LINE_SPACING, color='0.75', lw=0.6, ls='--')
    if recorder.planned:
        planned = np.array(recorder.planned)
        axes[0].plot(planned[:, 0], planned[:, 1], lw=0.8, color='C2', label='planned path')
    axes[0].plot(track[:, 1], track[:, 2], lw=1.2, color='C0', label='rover')
    for name, (centre_x, centre_y, half_x, half_y) in OBSTACLES.items():
        shape = (plt.Circle((centre_x, centre_y), half_x, color='C3', alpha=0.6) if half_x == half_y
                 else plt.Rectangle((centre_x - half_x, centre_y - half_y), 2 * half_x, 2 * half_y,
                                    color='C3', alpha=0.6))
        axes[0].add_patch(shape)
        # A removed bystander stops existing, so only samples taken while it stood there count
        present = (track[:, 0] < recorder.bystander_left
                   if name == (BYSTANDER or {}).get('name') and recorder.bystander_left
                   else np.ones(len(track), dtype=bool))
        clearance = np.array([surface_distance(point, (centre_x, centre_y, half_x, half_y))
                              for point in track[present, 1:3]]) - SWING_RADIUS
        axes[2].plot(track[present, 0] - start, clearance, lw=0.8, label=name)
        print(f'{name:20s} minimum body clearance {clearance.min():+.2f} m')
    axes[0].set(xlabel='east [m]', ylabel='north [m]', title='Track around the obstacles', xlim=(-2, 55))
    axes[0].axis('equal')
    axes[0].legend(loc='upper left')

    axes[1].plot(commands[:, 0] - start, commands[:, 2], lw=0.7, color='C1')
    if recorder.bystander_left is not None:
        for axis in axes[1:]:
            axis.axvline(recorder.bystander_left - start, color='0.4', ls=':', lw=0.8)
    axes[1].axhline(1.5, color='C3', ls='--', lw=0.7, label='CAN cap')
    axes[1].axhline(-1.5, color='C3', ls='--', lw=0.7)
    axes[1].set(xlabel='time [s]', ylabel='commanded yaw rate [rad/s]', title='Steering smoothness')
    axes[1].legend()

    axes[2].axhline(0.0, color='C3', lw=0.8)
    axes[2].set(xlabel='time [s]', ylabel='body clearance [m]', title='Clearance to each obstacle', ylim=(-1, 6))
    axes[2].legend()

    driving = commands[:, 1] > 0.5
    print(f'yaw rate while driving: max {np.abs(commands[driving, 2]).max():.2f} rad/s  '
          f'rms {np.sqrt(np.mean(commands[driving, 2] ** 2)):.2f} rad/s')
    # Survey data lost to detours: distance driven more than 0.2 m off the line, on the lines only
    on_lines = (track[:, 1] > 1.0) & (track[:, 1] < LINE_LENGTH - 1.0)
    cross_track = np.abs(track[:, 2] - np.round(track[:, 2] / LINE_SPACING) * LINE_SPACING)
    steps = np.hypot(np.diff(track[:, 1]), np.diff(track[:, 2]))
    off = (cross_track[1:] > 0.2) & on_lines[1:]
    print(f'off-line distance (|cross-track| > 0.2 m on the lines): {steps[off].sum():.1f} m  '
          f'of {steps[on_lines[1:]].sum():.1f} m on lines; max cross-track {cross_track[on_lines].max():.2f} m')
    np.savez(filename.replace('.png', '.npz'), track=track, commands=commands)
    print(f'track samples {len(track)}  final position ({track[-1, 1]:.1f}, {track[-1, 2]:.1f})')
    figure.tight_layout()
    figure.savefig(filename, dpi=150)
    print(f'wrote {filename}')


def main():
    seconds = float(sys.argv[1])
    filename = sys.argv[2]
    rclpy.init()
    recorder = DetourRecorder()

    start = recorder.get_clock().now()
    while (recorder.get_clock().now() - start).nanoseconds < seconds * 1e9:
        rclpy.spin_once(recorder, timeout_sec=0.05)

    report(recorder, filename)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
