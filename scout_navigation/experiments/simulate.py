import numpy as np

from scout_navigation.path_tracking import curvature_limited_speed
from scout_navigation.rover_model import DifferentialDriveRover
from scout_navigation.survey_grid import generate_boustrophedon_path

SEARCH_WINDOW = 50


class GnssReceiver:
    """White noise on a slowly drifting bias, matching the error budget of a receiver grade."""

    def __init__(self, noise_std, bias_std, bias_time_constant, rng):
        self.noise_std = noise_std
        self.bias_std = bias_std
        self.bias_time_constant = bias_time_constant
        self.rng = rng
        self.bias = rng.normal(0.0, bias_std, 2)

    def measure(self, position, time_step):
        decay = np.exp(-time_step / self.bias_time_constant)
        self.bias = decay * self.bias + np.sqrt(1.0 - decay ** 2) * self.rng.normal(
            0.0, self.bias_std, 2)
        return position + self.bias + self.rng.normal(0.0, self.noise_std, 2)


class HeadingSensor:
    """Yaw estimate with white noise and an unbounded drift rate."""

    def __init__(self, noise_std, drift_rate, rng):
        self.noise_std = noise_std
        self.drift_rate = drift_rate
        self.rng = rng
        self.drift = 0.0

    def measure(self, heading, time_step):
        self.drift += self.drift_rate * time_step * self.rng.normal()
        return heading + self.drift + self.rng.normal(0.0, self.noise_std)


def build_rover():
    return DifferentialDriveRover(max_linear_speed=1.5, max_angular_speed=1.5,
                                  max_linear_acceleration=0.5, max_angular_acceleration=1.5,
                                  yaw_slip_factor=0.1)


def build_path(turn_radius, point_spacing=0.1):
    return generate_boustrophedon_path(width=50.0, length=50.0, line_spacing=5.0,
                                       turn_radius=turn_radius, point_spacing=point_spacing)


def simulate(path, controller, rover, gnss, heading_sensor, cruise_speed, time_step,
             max_lateral_acceleration=0.5, lookahead_samples=50, max_steps=20000):
    """Runs the tracking loop on measured state and scores it against the true position."""
    measured_index = 0
    true_index = 0
    trajectory, errors, speeds, yaw_rates, on_line = [], [], [], [], []

    for _ in range(max_steps):
        measured_position = gnss.measure(rover.position, time_step)
        measured_heading = heading_sensor.measure(rover.heading, time_step)
        measured_index = path.nearest_index(measured_position, measured_index, SEARCH_WINDOW)

        speed = curvature_limited_speed(path, measured_index, cruise_speed,
                                        max_lateral_acceleration, lookahead_samples)
        angular_velocity = controller.angular_velocity(
            measured_position, measured_heading, rover.linear_speed, path, measured_index)
        rover.step(speed, angular_velocity, time_step)

        true_index = path.nearest_index(rover.position, true_index, SEARCH_WINDOW)
        trajectory.append(rover.position.copy())
        errors.append(path.cross_track_error(rover.position, true_index))
        speeds.append(rover.linear_speed)
        yaw_rates.append(rover.angular_speed)
        on_line.append(not path.turn_flags[true_index])

        if true_index >= len(path) - 2:
            break

    return {'trajectory': np.array(trajectory), 'cross_track_error': np.array(errors),
            'speed': np.array(speeds), 'yaw_rate': np.array(yaw_rates),
            'on_line': np.array(on_line),
            'duration': len(errors) * time_step}


def line_error_metrics(result):
    """Cross-track statistics over survey lines only, where the 10% spacing spec applies."""
    errors = np.abs(result['cross_track_error'][result['on_line']])
    lateral_acceleration = result['speed'] * result['yaw_rate']
    return {'rms': float(np.sqrt(np.mean(errors ** 2))), 'max': float(errors.max()),
            'p95': float(np.percentile(errors, 95)),
            'yaw_rate_rms': float(np.sqrt(np.mean(result['yaw_rate'] ** 2))),
            'lateral_acceleration_rms': float(np.sqrt(np.mean(lateral_acceleration ** 2))),
            'duration': result['duration']}
