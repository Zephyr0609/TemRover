import numpy as np


def arc(centre, radius, start_angle, sweep, segment_length):
    """Points along a circular arc, excluding its first point; positive sweep is anticlockwise."""
    count = max(int(np.ceil(abs(sweep) * radius / segment_length)), 1)
    angles = start_angle + sweep * np.arange(1, count + 1) / count
    return centre + radius * np.column_stack([np.cos(angles), np.sin(angles)])


def u_turn(spacing, radius, segment_length):
    """Left U-turn from (0, 0) heading +x to (0, spacing) heading -x, in that local frame.

    Two quarter arcs joined by a straight when the radius fits half the spacing, otherwise a bulb:
    swing out right, a wide left arc, swing back right, all at the same radius.
    """
    if 2.0 * radius <= spacing:
        first = arc(np.array([0.0, radius]), radius, -np.pi / 2.0, np.pi / 2.0, segment_length)
        across_end = np.array([radius, spacing - radius])
        count = int(np.ceil((spacing - 2.0 * radius) / segment_length))
        across = first[-1] + np.outer(np.arange(1, count + 1) / count, across_end - first[-1])
        last = arc(np.array([0.0, spacing - radius]), radius, 0.0, np.pi / 2.0, segment_length)
        return np.vstack([first, across, last])

    out_centre = np.array([0.0, -radius])
    back_centre = np.array([0.0, spacing + radius])
    big_centre = np.array([np.sqrt(4.0 * radius ** 2 - (radius + spacing / 2.0) ** 2), spacing / 2.0])
    swing = np.arctan2(big_centre[0], big_centre[1] - out_centre[1])
    entry = np.arctan2(*(out_centre - big_centre)[::-1])
    exit_ = np.arctan2(*(back_centre - big_centre)[::-1])
    return np.vstack([
        arc(out_centre, radius, np.pi / 2.0, -swing, segment_length),
        arc(big_centre, radius, entry, (exit_ - entry) % (2.0 * np.pi), segment_length),
        arc(back_centre, radius, exit_ + np.pi, -swing, segment_length),
    ])


def generate_survey_mission(width, length, line_spacing, bearing, origin,
                            turn_radius, headland, segment_length):
    """Survey lines joined by spot turns, or by driven U-turns when a turn radius is given.

    With a towed train every line is driven `headland` metres past both ends, so the carts are
    straight on the line before it starts and the last cart clears it before the rover turns.
    """
    rotation = np.array([[np.cos(bearing), -np.sin(bearing)], [np.sin(bearing), np.cos(bearing)]])
    place = (lambda points: origin + points @ rotation.T)
    line_count = int(round(width / line_spacing)) + 1
    steps = []

    for line in range(line_count):
        forward = line % 2 == 0
        y = line * line_spacing
        start = np.array([-headland if forward else length + headland, y])
        end = np.array([length + headland if forward else -headland, y])
        steps.append(('drive', place(start), place(end)))

        if line == line_count - 1:
            break

        if turn_radius == 0.0:
            quarter = np.pi / 2.0 if forward else -np.pi / 2.0
            steps.append(('turn', quarter))
            steps.append(('headland', place(end), place(end + np.array([0.0, line_spacing]))))
            steps.append(('turn', quarter))
            continue

        # Mirror the left U-turn for lines driven towards -x, then lay it at the line end
        local = u_turn(line_spacing, turn_radius, segment_length)
        path = end + (local if forward else local * np.array([-1.0, 1.0]))
        points = np.vstack([end, path])
        steps.extend(('headland', place(a), place(b)) for a, b in zip(points[:-1], points[1:]))

    return steps
