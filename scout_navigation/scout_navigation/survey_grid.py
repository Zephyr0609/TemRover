import numpy as np

from .geodesy import wrap_to_pi

ROTATION_STEP = np.pi / 4.0


class SurveyPath:
    """Densely sampled path with per-sample heading, line index and turn flag."""

    def __init__(self, points, headings, line_indices, turn_flags):
        self.points = points
        self.headings = headings
        self.line_indices = line_indices
        self.turn_flags = turn_flags

        steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
        turn_rates = wrap_to_pi(np.diff(headings))
        # A large step is an in-place rotation, not curvature the rover drives through
        # A large heading step is an in-place rotation, not curvature the rover drives through
        turn_rates = np.where(np.abs(turn_rates) > ROTATION_STEP, 0.0, turn_rates / steps)
        self.curvatures = np.append(turn_rates, turn_rates[-1])

    def __len__(self):
        return len(self.points)

    def nearest_index(self, position, previous_index, window):
        """Searches forward only — a teardrop turn passes close to itself and would match backwards."""
        stop = min(previous_index + window, len(self.points))
        offsets = self.points[previous_index:stop] - position
        return previous_index + int(np.argmin(np.einsum('ij,ij->i', offsets, offsets)))

    def cross_track_error(self, position, index):
        offset = position - self.points[index]
        left_normal = np.array([-np.sin(self.headings[index]), np.cos(self.headings[index])])
        return float(offset @ left_normal)


def _arc(entry, heading, radius, swept_angle, point_spacing):
    """Constant-curvature arc sampled from the entry pose, the last sample being the exit pose."""
    quarter_turn = np.sign(swept_angle) * np.pi / 2.0
    centre = entry + radius * np.array([np.cos(heading + quarter_turn), np.sin(heading + quarter_turn)])

    count = max(int(abs(swept_angle) * radius / point_spacing), 1)
    angles = heading - quarter_turn + np.linspace(0.0, swept_angle, count + 1)
    return centre + radius * np.stack([np.cos(angles), np.sin(angles)], axis=-1), angles + quarter_turn


def _straight(entry, heading, length, point_spacing):
    count = max(int(length / point_spacing), 1)
    direction = np.array([np.cos(heading), np.sin(heading)])
    return entry + np.outer(np.linspace(0.0, length, count + 1), direction), np.full(count + 1, heading)


def _join(segments):
    """Concatenates segments, dropping the pose each one shares with the next."""
    points = [segment[0][:-1] for segment in segments[:-1]] + [segments[-1][0]]
    headings = [segment[1][:-1] for segment in segments[:-1]] + [segments[-1][1]]
    return np.concatenate(points), np.concatenate(headings)


def square_turn(entry, heading, spacing, radius, point_spacing, turn_sign):
    """Crosses to the next line between two in-place rotations — the Scout can spin on the spot."""
    cross_heading = wrap_to_pi(heading + turn_sign * np.pi / 2.0)
    points, headings = _straight(entry, cross_heading, spacing, point_spacing)
    return points, np.concatenate([headings[:-1], [wrap_to_pi(heading + np.pi)]])


def pi_turn(entry, heading, spacing, radius, point_spacing, turn_sign):
    """Quarter arc, straight, quarter arc — available when the lines are at least 2R apart."""
    first = _arc(entry, heading, radius, turn_sign * np.pi / 2.0, point_spacing)
    bridge = _straight(first[0][-1], first[1][-1], spacing - 2.0 * radius, point_spacing)
    second = _arc(bridge[0][-1], bridge[1][-1], radius, turn_sign * np.pi / 2.0, point_spacing)
    return _join([first, bridge, second])


def teardrop_turn(entry, heading, spacing, radius, point_spacing, turn_sign):
    """Swings away by arccos(spacing / 2R) first — the forward-only turn for lines closer than 2R."""
    away_angle = np.arccos(spacing / (2.0 * radius))
    first = _arc(entry, heading, radius, -turn_sign * away_angle, point_spacing)
    second = _arc(first[0][-1], first[1][-1], radius, turn_sign * (np.pi + away_angle), point_spacing)
    return _join([first, second])


def generate_boustrophedon_path(width, length, line_spacing, turn_radius, point_spacing):
    """Lawnmower coverage of a width x length rectangle, the first line running +x from the origin."""
    line_count = int(round(width / line_spacing)) + 1
    if turn_radius <= 0.0:
        turn = square_turn
    elif line_spacing >= 2.0 * turn_radius:
        turn = pi_turn
    else:
        turn = teardrop_turn

    segments = []
    cursor, heading = np.zeros(2), 0.0

    for line in range(line_count):
        line_end = length if line % 2 == 0 else 0.0
        straight = _straight(cursor, heading, abs(line_end - cursor[0]), point_spacing)
        segments.append((straight[0], straight[1], line, False))
        cursor, heading = straight[0][-1], straight[1][-1]

        if line == line_count - 1:
            break

        turn_sign = 1.0 if line % 2 == 0 else -1.0
        curve = turn(cursor, heading, line_spacing, turn_radius, point_spacing, turn_sign)
        segments.append((curve[0], curve[1], line, True))
        cursor, heading = curve[0][-1], wrap_to_pi(curve[1][-1])

    trimmed = [(points[:-1], headings[:-1], line, is_turn)
               for points, headings, line, is_turn in segments[:-1]] + segments[-1:]

    return SurveyPath(
        np.concatenate([points for points, _, _, _ in trimmed]),
        wrap_to_pi(np.concatenate([headings for _, headings, _, _ in trimmed])),
        np.concatenate([np.full(len(points), line) for points, _, line, _ in trimmed]),
        np.concatenate([np.full(len(points), is_turn) for points, _, _, is_turn in trimmed]))


def rotate_path(path, angle):
    """Rotates a grid-frame path into the local ENU frame."""
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return SurveyPath(path.points @ rotation.T, wrap_to_pi(path.headings + angle),
                      path.line_indices, path.turn_flags)


def generate_survey_mission(width, length, line_spacing, bearing, origin):
    """Straight runs joined by quarter turns, each turn relative so headings never accumulate."""
    rotation = np.array([[np.cos(bearing), -np.sin(bearing)], [np.sin(bearing), np.cos(bearing)]])
    line_count = int(round(width / line_spacing)) + 1
    steps = []
    cursor = np.zeros(2)

    for line in range(line_count):
        line_end = np.array([length if line % 2 == 0 else 0.0, line * line_spacing])
        steps.append(('drive', origin + rotation @ cursor, origin + rotation @ line_end))
        cursor = line_end

        if line == line_count - 1:
            break

        quarter = np.pi / 2.0 if line % 2 == 0 else -np.pi / 2.0
        across = cursor + np.array([0.0, line_spacing])
        steps.append(('turn', quarter))
        steps.append(('drive', origin + rotation @ cursor, origin + rotation @ across))
        steps.append(('turn', quarter))
        cursor = across

    return steps
