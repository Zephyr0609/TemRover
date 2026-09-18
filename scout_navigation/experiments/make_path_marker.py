"""Writes the planned survey path as a static Gazebo model so it is visible in the scene."""
import numpy as np

from scout_navigation.geodesy import wrap_to_pi
from scout_navigation.survey_grid import generate_boustrophedon_path
from survey_config import load_grid

GRID = load_grid()
RIBBON_WIDTH = 0.12
RIBBON_HEIGHT = 0.02
HEADING_TOLERANCE = np.radians(10.0)
MARKER_RADIUS = 0.45


def segments(path):
    """Breaks the path wherever the heading has turned past the tolerance, then draws chords."""
    bounds = [0]
    for index in range(1, len(path)):
        if abs(wrap_to_pi(path.headings[index] - path.headings[bounds[-1]])) > HEADING_TOLERANCE:
            bounds.append(index)
    bounds.append(len(path) - 1)

    for start, stop in zip(bounds[:-1], bounds[1:]):
        offset = path.points[stop] - path.points[start]
        length = float(np.hypot(*offset))
        if length < RIBBON_WIDTH:
            continue
        centre = (path.points[start] + path.points[stop]) / 2.0
        yield centre, length, float(np.arctan2(offset[1], offset[0]))


def ribbon(index, centre, length, heading):
    return f'''    <link name="segment_{index}">
      <pose>{centre[0]:.3f} {centre[1]:.3f} {RIBBON_HEIGHT / 2:.3f} 0 0 {heading:.4f}</pose>
      <visual name="visual">
        <geometry><box><size>{length:.3f} {RIBBON_WIDTH} {RIBBON_HEIGHT}</size></box></geometry>
        <material>
          <ambient>0.85 0.85 0.85 1</ambient>
          <diffuse>0.95 0.95 0.95 1</diffuse>
          <emissive>0.35 0.35 0.35 1</emissive>
        </material>
      </visual>
    </link>
'''


def endpoint(name, point, colour):
    return f'''    <link name="{name}">
      <pose>{point[0]:.3f} {point[1]:.3f} {MARKER_RADIUS:.3f} 0 0 0</pose>
      <visual name="visual">
        <geometry><sphere><radius>{MARKER_RADIUS}</radius></sphere></geometry>
        <material>
          <ambient>{colour} 1</ambient>
          <diffuse>{colour} 1</diffuse>
          <emissive>{colour} 1</emissive>
        </material>
      </visual>
    </link>
'''


def main():
    path = generate_boustrophedon_path(**GRID)
    pieces = [ribbon(index, *segment) for index, segment in enumerate(segments(path))]
    pieces.append(endpoint('start_marker', path.points[0], '0.15 0.65 0.25'))
    pieces.append(endpoint('end_marker', path.points[-1], '0.85 0.30 0.10'))

    with open('worlds/survey_path.sdf', 'w') as sdf:
        sdf.write('<?xml version="1.0"?>\n<sdf version="1.9">\n'
                  '  <model name="survey_path">\n    <static>true</static>\n')
        sdf.writelines(pieces)
        sdf.write('  </model>\n</sdf>\n')

    print(f'{len(pieces) - 2} ribbon segments, '
          f'start {path.points[0].round(2)}, end {path.points[-1].round(2)}')


if __name__ == '__main__':
    main()
