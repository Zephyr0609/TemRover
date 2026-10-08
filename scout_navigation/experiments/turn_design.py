"""Chooses the U-turn radius and headland run for the towed train from its hitch geometry.

The rover follows the planned turn exactly; each cart yaws about its off-axle pivot so its axle
follows the pivot's motion. Reports, per radius, the largest hitch angles and how far past the
line end the rover must drive so the Rx cart is back on the next line within tolerance.
"""
import sys

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import yaml
from scout_navigation.survey_grid import u_turn
from survey_config import CONFIG_DIRECTORY, parameters

PAYLOAD = yaml.safe_load((CONFIG_DIRECTORY / 'payload.yaml').read_text())['/**']['ros__parameters']
OFFSETS, LENGTHS = PAYLOAD['hitch_offsets'], PAYLOAD['trailer_lengths']
SPACING = parameters()['line_spacing']
STEP = 0.02
LATERAL_TOLERANCE = 0.05
HEADING_TOLERANCE = np.radians(1.0)
RADII = [2.5, 3.0, 3.5, 4.0, 5.0, 6.0]
RUN = 40.0


def rover_path(radius):
    """Straight up to the turn at the origin, the U-turn, then straight back along the next line."""
    approach = np.column_stack([np.arange(-RUN, 0.0, STEP), np.zeros(int(RUN / STEP))])
    turn = np.vstack([[0.0, 0.0], u_turn(SPACING, radius, STEP)])
    away = np.column_stack([-np.arange(STEP, RUN, STEP), np.full(int(RUN / STEP) - 1, SPACING)])
    return np.vstack([approach, turn, away])


def tow(path):
    """Unit poses along the path: rover, then each cart behind its pivot."""
    headings = np.arctan2(*np.diff(path, axis=0).T[::-1])
    poses = [np.column_stack([path[1:], headings])]
    for offset, length in zip(OFFSETS, LENGTHS):
        front = poses[-1]
        pivot = front[:, :2] - offset * np.column_stack([np.cos(front[:, 2]), np.sin(front[:, 2])])
        heading = np.empty(len(pivot))
        heading[0] = front[0, 2]
        for i in range(1, len(pivot)):
            motion = pivot[i] - pivot[i - 1]
            heading[i] = heading[i - 1] + np.hypot(*motion) / length * np.sin(
                np.arctan2(motion[1], motion[0]) - heading[i - 1])
        axle = pivot - length * np.column_stack([np.cos(heading), np.sin(heading)])
        poses.append(np.column_stack([axle, heading]))
    return poses


def assess(radius):
    poses = tow(rover_path(radius))
    hitch = [np.abs(np.angle(np.exp(1j * (a[:, 2] - b[:, 2])))).max() for a, b in zip(poses[:-1], poses[1:])]
    rx = poses[-1]
    returning = np.flatnonzero(np.cos(rx[:, 2]) < 0.0)
    returning = returning[returning > np.argmax(rx[:, 1] > SPACING / 2.0)]
    error = (np.abs(rx[returning, 1] - SPACING) > LATERAL_TOLERANCE) | (
        np.abs(np.angle(np.exp(1j * (rx[returning, 2] - np.pi)))) > HEADING_TOLERANCE)
    settled_x = rx[returning[np.flatnonzero(error)[-1] + 1], 0]
    train = sum(OFFSETS) + sum(LENGTHS)
    # The run past the line end must cover both: Rx clearing the line before the turn, Rx settled after it
    headland = max(train, -settled_x)
    return np.degrees(hitch), headland, -settled_x, poses


def main():
    print(f'train: pivots {OFFSETS} m behind, carts {LENGTHS} m to axle, '
          f'rover to Rx axle {sum(OFFSETS) + sum(LENGTHS):.2f} m')
    print(' radius  rover-Tx  Tx-Rx   Rx settled   headland run')
    results = {}
    for radius in RADII:
        hitch, headland, settled, poses = assess(radius)
        results[radius] = poses
        print(f'{radius:6.1f} m  {hitch[0]:5.1f} deg  {hitch[1]:5.1f} deg  {settled:6.1f} m   {headland:6.1f} m')

    figure, axes = plt.subplots(1, 2, figsize=(14, 6))
    for axis, radius in zip(axes, (RADII[0], float(sys.argv[1]) if len(sys.argv) > 1 else 4.0)):
        poses = results[radius]
        for line in (0.0, SPACING):
            axis.axhline(line, color='0.7', ls='--', lw=0.8)
        for (colour, label), pose in zip((('C0', 'rover'), ('C1', 'Tx axle'), ('C3', 'Rx axle')), poses):
            axis.plot(pose[:, 0], pose[:, 1], color=colour, lw=1.4, label=label)
        axis.set(title=f'U-turn radius {radius} m, line spacing {SPACING} m', xlabel='along line [m]',
                 ylabel='across [m]', xlim=(-20, 16))
        axis.set_aspect('equal')
        axis.legend(loc='lower left')
    figure.tight_layout()
    figure.savefig('results/u_turn_design.png', dpi=140)
    print('wrote results/u_turn_design.png')


if __name__ == '__main__':
    main()
