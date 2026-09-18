"""Shows why a semicircle cannot close a 5 m line spacing once the turn radius exceeds 2.5 m."""
import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np

from scout_navigation.survey_grid import _arc, teardrop_turn

LINE_SPACING = 5.0
POINT_SPACING = 0.05
ENTRY = np.array([0.0, 0.0])
TRAIN_LENGTH = 7.58


def semicircle(radius):
    return _arc(ENTRY, 0.0, radius, np.pi, POINT_SPACING)[0]


def main():
    figure, axes = plt.subplots(figsize=(9.0, 6.0))

    for line in (0.0, LINE_SPACING, 2.0 * LINE_SPACING):
        axes.axhline(line, color='0.78', lw=1.0, zorder=0)

    tight = semicircle(LINE_SPACING / 2.0)
    wide = semicircle(4.0)
    drop = teardrop_turn(ENTRY, 0.0, LINE_SPACING, 4.0, POINT_SPACING, 1.0)[0]

    for track, colour, label, anchor in (
            (tight, 'C0', 'semicircle\nR = 2.5 m', (1.1, 2.4)),
            (wide, 'C3', 'semicircle\nR = 4.0 m', (5.6, 1.8)),
            (drop, 'C2', 'teardrop\nR = 4.0 m', (10.7, 3.4))):
        axes.plot(track[:, 0], track[:, 1], color=colour, lw=2.2)
        axes.plot(track[-1, 0], track[-1, 1], 'o', color=colour, ms=7)
        axes.text(*anchor, label, color=colour, fontsize=11, ha='center', va='center')

    overshoot = 2.0 * 4.0 - LINE_SPACING
    axes.annotate('', xy=(1.4, 2.0 * 4.0), xytext=(1.4, LINE_SPACING),
                  arrowprops=dict(arrowstyle='<->', color='C3', lw=1.5))
    axes.text(1.1, LINE_SPACING + overshoot / 2.0, f'+{overshoot:.0f} m', color='C3',
              fontsize=12, va='center', ha='right')

    axes.plot([ENTRY[0], ENTRY[0] - TRAIN_LENGTH], [0.0, 0.0], color='0.25', lw=4.0,
              solid_capstyle='butt', zorder=3)
    axes.text(ENTRY[0] - TRAIN_LENGTH / 2.0, -0.9, f'{TRAIN_LENGTH:.1f} m train', fontsize=10,
              color='0.25', ha='center')

    axes.set(xlabel='along track [m]', ylabel='across track [m]')
    axes.set_title('Row-end turn at 5 m line spacing', fontsize=14)
    axes.text(-9.0, 9.4, 'a semicircle lands at 2R', fontsize=11, color='0.45')
    axes.set_xlim(-9.5, 14.0)
    axes.set_ylim(-3.4, 10.6)
    axes.set_aspect('equal')
    figure.tight_layout()
    figure.savefig('results/turn_geometry.png', dpi=160)
    print('semicircle R=2.5 lands at y =', round(float(tight[-1, 1]), 3))
    print('semicircle R=4.0 lands at y =', round(float(wide[-1, 1]), 3))
    print('teardrop   R=4.0 lands at y =', round(float(drop[-1, 1]), 3))


if __name__ == '__main__':
    main()
