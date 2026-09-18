import sys

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from scout_navigation.geodesy import wrap_to_pi
from scout_navigation.survey_grid import generate_boustrophedon_path, rotate_path
from survey_config import grid_bearing, load_grid

MODEL_NAME = 'temrover'
TRACKED_LINKS = {'tx_cart_link': 'Tx cart', 'rx_cart_link': 'Rx cart'}
SITE = sys.argv[2] if len(sys.argv) > 2 else None
GRID = load_grid(SITE)
BEARING = grid_bearing(SITE)


def roll_pitch(orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return roll, pitch


class RunRecorder(Node):
    """Logs rover and cart poses against the planned survey line."""

    def __init__(self):
        super().__init__('run_recorder')
        self.path = rotate_path(generate_boustrophedon_path(**GRID), BEARING)
        self.samples = {name: [] for name in ['rover'] + list(TRACKED_LINKS.values())}
        self.create_subscription(TFMessage, 'link_poses', self.on_poses, 10)

    def record(self, label, position, height, rotation):
        index = self.path.nearest_index(position, 0, len(self.path))
        roll, pitch = roll_pitch(rotation)
        self.samples[label].append((position[0], position[1],
                                    self.path.cross_track_error(position, index),
                                    float(self.path.turn_flags[index]), roll, pitch, height))

    def on_poses(self, message):
        """Link poses arrive in the model frame, so compose them with the model pose."""
        poses = {transform.child_frame_id: transform.transform for transform in message.transforms}
        model = poses.get(MODEL_NAME)
        if model is None:
            return

        origin = np.array([model.translation.x, model.translation.y])
        yaw = 2.0 * np.arctan2(model.rotation.z, model.rotation.w)
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        self.record('rover', origin, model.translation.z, model.rotation)

        for frame, label in TRACKED_LINKS.items():
            link = poses.get(frame)
            if link is None:
                continue
            offset = rotation @ np.array([link.translation.x, link.translation.y])
            self.record(label, origin + offset,
                        model.translation.z + link.translation.z, link.rotation)


def geometry_metrics(samples):
    """Tx-Rx separation and height errors, matching the Phase-1 Simscape metrics."""
    tx = np.array(samples['Tx cart'])
    rx = np.array(samples['Rx cart'])
    count = min(len(tx), len(rx))
    tx, rx = tx[:count], rx[:count]

    separation = np.hypot(tx[:, 0] - rx[:, 0], tx[:, 1] - rx[:, 1])
    return separation - separation[0], tx[:, 6] - rx[:, 6]


def report(recorder, filename):
    figure, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].plot(recorder.path.points[:, 0], recorder.path.points[:, 1], lw=0.7, color='0.6',
                 label='planned line')

    for label, rows in ((name, rows) for name, rows in recorder.samples.items() if rows):
        data = np.array(rows)
        axes[0].plot(data[:, 0], data[:, 1], lw=1.0, label=label)

        on_line = data[:, 3] < 0.5
        error = np.abs(data[on_line, 2])
        tilt = np.degrees(np.abs(data[:, 4:6])).max(axis=1)
        print(f'{label:9s} cross-track rms {np.sqrt(np.mean(error ** 2)):.3f} m  '
              f'max {error.max():.3f} m   tilt max {tilt.max():.1f} deg')
        axes[1].plot(np.abs(data[:, 2]), lw=0.7, label=label)

    axes[0].set(xlabel='east [m]', ylabel='north [m]', title='Towed system track')
    axes[0].axis('equal')
    axes[0].legend()
    if not (recorder.samples['Tx cart'] and recorder.samples['Rx cart']):
        finish(figure, axes, filename)
        return

    separation_error, height_error = geometry_metrics(recorder.samples)
    print(f'Tx-Rx     separation error rms {np.sqrt(np.mean(separation_error ** 2)):.3f} m  '
          f'max {np.abs(separation_error).max():.3f} m   (dr tolerance 0.05 m)')
    print(f'Tx-Rx     height error     rms {np.sqrt(np.mean(height_error ** 2)):.3f} m  '
          f'max {np.abs(height_error).max():.3f} m   (dz tolerance 0.05 m)')

    finish(figure, axes, filename)


def finish(figure, axes, filename):
    axes[1].axhline(0.5, color='C3', ls='--', label='0.5 m spec')
    axes[1].set(xlabel='sample', ylabel='|cross-track| [m]', title='Deviation from the survey line')
    axes[1].legend()
    figure.tight_layout()
    figure.savefig(filename, dpi=150)
    print(f'wrote {filename}')


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    recorder = RunRecorder()

    start = recorder.get_clock().now()
    while (recorder.get_clock().now() - start).nanoseconds < seconds * 1e9:
        rclpy.spin_once(recorder, timeout_sec=0.05)

    report(recorder, sys.argv[3] if len(sys.argv) > 3 else 'results/towed_system.png')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
