import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np

from scout_navigation.path_tracking import PurePursuitController, StanleyController
from simulate import GnssReceiver, HeadingSensor, build_path, build_rover, line_error_metrics, simulate

TIME_STEP = 0.05
CRUISE_SPEED = 1.5
SPEC_LIMIT = 0.5
TURN_RADIUS = 2.0
POINT_SPACING = 0.1

GNSS_GRADES = {
    'RTK': dict(noise_std=0.02, bias_std=0.01, bias_time_constant=60.0),
    'SBAS': dict(noise_std=0.35, bias_std=0.4, bias_time_constant=300.0),
    'M9N standalone': dict(noise_std=0.8, bias_std=1.0, bias_time_constant=300.0),
}
HEADING_GRADE = dict(noise_std=np.radians(2.0), drift_rate=0.001)

CONTROLLER_CANDIDATES = {
    'stanley': [StanleyController(gain, 0.5, rate)
                for gain in (1.0, 2.0, 4.0) for rate in (0.5, 1.0, 2.0)],
    'pure pursuit': [PurePursuitController(gain, minimum, POINT_SPACING)
                     for gain in (0.5, 1.0, 2.0) for minimum in (1.0, 1.5, 2.5)],
}


def run_case(path, controller, grade, seed):
    rng = np.random.default_rng(seed)
    return simulate(path, controller, build_rover(), GnssReceiver(rng=rng, **grade),
                    HeadingSensor(rng=rng, **HEADING_GRADE), CRUISE_SPEED, TIME_STEP)


def best_controller(path, candidates, grade, seed):
    """Selects the candidate with the lowest 95th-percentile line error for this receiver grade."""
    scored = [(line_error_metrics(run_case(path, candidate, grade, seed)), candidate)
              for candidate in candidates]
    return min(scored, key=lambda entry: entry[0]['p95'])


def plot_tracking(path, result, filename):
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(path.points[:, 0], path.points[:, 1], lw=0.8, color='0.6', label='planned')
    axes[0].plot(result['trajectory'][:, 0], result['trajectory'][:, 1], lw=0.8, color='C0',
                 label='driven')
    axes[0].set(xlabel='east [m]', ylabel='north [m]', title='Survey coverage')
    axes[0].legend()
    axes[0].axis('equal')

    time = np.arange(len(result['cross_track_error'])) * TIME_STEP
    axes[1].plot(time, result['cross_track_error'], lw=0.6, color='C0')
    axes[1].fill_between(time, -SPEC_LIMIT, SPEC_LIMIT, color='C2', alpha=0.15,
                         label=f'+/-{SPEC_LIMIT} m spec')
    axes[1].plot(time[~result['on_line']], result['cross_track_error'][~result['on_line']], '.',
                 ms=1.5, color='C3', label='in turn')
    axes[1].set(xlabel='time [s]', ylabel='cross-track error [m]', title='Line tracking error')
    axes[1].legend()
    figure.tight_layout()
    figure.savefig(filename, dpi=150)


def plot_grade_requirement(table, filename):
    figure, axis = plt.subplots(figsize=(7, 4.5))
    grades = list(GNSS_GRADES)
    width = 0.35

    for offset, (name, metrics) in zip((-width / 2, width / 2), table.items()):
        positions = np.arange(len(grades)) + offset
        axis.bar(positions, [metrics[grade]['p95'] for grade in grades], width, label=f'{name} p95')
        axis.plot(positions, [metrics[grade]['max'] for grade in grades], 'k_', ms=14,
                  label='max' if offset < 0 else None)

    axis.axhline(SPEC_LIMIT, color='C3', ls='--', label=f'{SPEC_LIMIT} m spec')
    axis.set(xticks=np.arange(len(grades)), ylabel='line cross-track error [m]',
             title='Receiver grade needed for 10% of 5 m line spacing')
    axis.set_xticklabels(grades)
    axis.legend()
    figure.tight_layout()
    figure.savefig(filename, dpi=150)


def main():
    path = build_path(TURN_RADIUS, POINT_SPACING)
    table = {}

    for name, candidates in CONTROLLER_CANDIDATES.items():
        table[name] = {}
        for grade_name, grade in GNSS_GRADES.items():
            metrics, controller = best_controller(path, candidates, grade, seed=0)
            table[name][grade_name] = metrics
            print(f'{name:13s} {grade_name:15s} rms={metrics["rms"]:.3f} p95={metrics["p95"]:.3f} '
                  f'max={metrics["max"]:.3f} yaw_rms={metrics["yaw_rate_rms"]:.3f} '
                  f'lat_acc_rms={metrics["lateral_acceleration_rms"]:.3f} '
                  f'duration={metrics["duration"]:.0f}s')

            if grade_name == 'RTK':
                plot_tracking(path, run_case(path, controller, grade, seed=0),
                              f'results/tracking_{name.replace(" ", "_")}.png')

    plot_grade_requirement(table, 'results/gnss_grade_requirement.png')


if __name__ == '__main__':
    main()
