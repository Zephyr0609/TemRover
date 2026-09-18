"""Overlays two recorded runs on the same obstacles and scores how far each strayed from the lines."""
import sys

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
from survey_config import load_obstacles

LINE_SPACING = 5.0
LINE_LENGTH = 50.0
LINES_SHOWN = 3
OFF_LINE = 0.2


def cross_track(track):
    return track[:, 2] - np.round(track[:, 2] / LINE_SPACING) * LINE_SPACING


def on_lines(track):
    return (track[:, 1] > 1.0) & (track[:, 1] < LINE_LENGTH - 1.0)


def score(track):
    """Distance driven off the line, and the worst excursion, counted on the survey lines only."""
    steps = np.hypot(np.diff(track[:, 1]), np.diff(track[:, 2]))
    lines = on_lines(track)[1:]
    off = (np.abs(cross_track(track))[1:] > OFF_LINE) & lines
    return steps[off].sum(), steps[lines].sum(), np.abs(cross_track(track))[on_lines(track)].max()


def draw_obstacles(axis):
    for item in load_obstacles():
        (x, y, _), (half_x, half_y) = item['pose'], item['half_size']
        shape = (plt.Circle((x, y), half_x, color='C3', alpha=0.7) if half_x == half_y
                 else plt.Rectangle((x - half_x, y - half_y), 2 * half_x, 2 * half_y, color='C3', alpha=0.7))
        axis.add_patch(shape)


def main():
    runs = {sys.argv[4]: np.load(sys.argv[1])['track'], sys.argv[5]: np.load(sys.argv[2])['track']}
    figure, axes = plt.subplots(2, 1, figsize=(13, 9), gridspec_kw={'height_ratios': [1.5, 1]})

    for line in range(LINES_SHOWN):
        axes[0].axhline(line * LINE_SPACING, color='0.6', lw=0.8, ls='--')
    draw_obstacles(axes[0])
    for (label, track), colour in zip(runs.items(), ('C0', 'C2')):
        shown = track[track[:, 2] < LINES_SHOWN * LINE_SPACING - 1.0]
        axes[0].plot(shown[:, 1], shown[:, 2], lw=1.6, color=colour, label=label)
        keep = on_lines(track) & (track[:, 2] < LINES_SHOWN * LINE_SPACING - 1.0)
        axes[1].plot(track[keep, 1], cross_track(track)[keep], '.', ms=2, color=colour, label=label)
        off, total, worst = score(track)
        print(f'{label:22s} off-line {off:6.1f} m of {total:6.1f} m on lines ({100 * off / total:4.1f} %)   '
              f'max cross-track {worst:.2f} m')

    axes[0].set(xlabel='east [m]', ylabel='north [m]', title='Track through the same seven obstacles',
                xlim=(-2, 55), ylim=(-3, LINES_SHOWN * LINE_SPACING))
    axes[0].set_aspect('equal')
    axes[0].legend(loc='upper right')
    axes[1].axhline(0.0, color='0.4', lw=0.8)
    axes[1].axhline(OFF_LINE, color='C3', lw=0.7, ls=':')
    axes[1].axhline(-OFF_LINE, color='C3', lw=0.7, ls=':')
    axes[1].set(xlabel='distance along the line [m]', ylabel='offset from the survey line [m]',
                title='Offset from the nearest survey line, lines 1–3', xlim=(0, LINE_LENGTH))
    axes[1].legend(loc='upper right')
    figure.tight_layout()
    figure.savefig(sys.argv[3], dpi=150)
    print(f'wrote {sys.argv[3]}')


if __name__ == '__main__':
    main()
