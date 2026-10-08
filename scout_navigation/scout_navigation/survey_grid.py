import numpy as np


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
