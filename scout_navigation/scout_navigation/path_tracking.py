import numpy as np

from .geodesy import wrap_to_pi


class StanleyController:
    """Front-axle law: heading error plus an arctan cross-track term, softened at low speed."""

    def __init__(self, cross_track_gain, softening_speed, steering_rate):
        self.cross_track_gain = cross_track_gain
        self.softening_speed = softening_speed
        self.steering_rate = steering_rate

    def angular_velocity(self, position, heading, speed, path, index):
        heading_error = wrap_to_pi(path.headings[index] - heading)
        cross_track_error = path.cross_track_error(position, index)
        correction = np.arctan2(-self.cross_track_gain * cross_track_error,
                                self.softening_speed + speed)
        return self.steering_rate * (heading_error + correction)


class PurePursuitController:
    """Chases a path point a speed-dependent distance ahead, optionally shifted sideways."""

    def __init__(self, lookahead_gain, minimum_lookahead, point_spacing):
        self.lookahead_gain = lookahead_gain
        self.minimum_lookahead = minimum_lookahead
        self.point_spacing = point_spacing

    def angular_velocity(self, position, heading, speed, path, index, lateral_offset=0.0):
        lookahead = max(self.lookahead_gain * speed, self.minimum_lookahead)
        target_index = min(index + int(lookahead / self.point_spacing), len(path) - 1)
        target_heading = path.headings[target_index]
        target = path.points[target_index] + lateral_offset * np.array(
            [-np.sin(target_heading), np.cos(target_heading)])
        offset = target - position
        bearing_error = wrap_to_pi(np.arctan2(offset[1], offset[0]) - heading)
        return 2.0 * speed * np.sin(bearing_error) / np.linalg.norm(offset)


def curvature_limited_speed(path, index, cruise_speed, max_lateral_acceleration, lookahead_samples):
    """Caps speed so lateral acceleration over the upcoming path stays within tolerance."""
    peak_curvature = np.abs(path.curvatures[index:index + lookahead_samples]).max()
    cruise_curvature = max_lateral_acceleration / cruise_speed ** 2
    return np.sqrt(max_lateral_acceleration / max(peak_curvature, cruise_curvature))
