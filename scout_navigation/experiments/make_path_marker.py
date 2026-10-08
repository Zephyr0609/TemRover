"""Writes the planned survey path as a static Gazebo model so it is visible in the scene."""
import numpy as np

from scout_navigation.survey_grid import generate_survey_mission
from survey_config import load_grid

GRID = load_grid()
RIBBON_WIDTH = 0.12
RIBBON_HEIGHT = 0.02
MARKER_RADIUS = 0.45


def segments(mission):
    """One ribbon per driven step of the mission, laid on the grid anchored at the origin."""
    for _, start, end in (step for step in mission if step[0] != 'turn'):
        offset = end - start
        yield (start + end) / 2.0, float(np.hypot(*offset)), float(np.arctan2(offset[1], offset[0]))


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
    mission = generate_survey_mission(**GRID, origin=np.zeros(2))
    drives = [step for step in mission if step[0] == 'drive']
    start, end = drives[0][1], drives[-1][2]
    pieces = [ribbon(index, *segment) for index, segment in enumerate(segments(mission))]
    pieces.append(endpoint('start_marker', start, '0.15 0.65 0.25'))
    pieces.append(endpoint('end_marker', end, '0.85 0.30 0.10'))

    with open('worlds/survey_path.sdf', 'w') as sdf:
        sdf.write('<?xml version="1.0"?>\n<sdf version="1.9">\n'
                  '  <model name="survey_path">\n    <static>true</static>\n')
        sdf.writelines(pieces)
        sdf.write('  </model>\n</sdf>\n')

    print(f'{len(pieces) - 2} ribbon segments, '
          f'start {start.round(2)}, end {end.round(2)}')


if __name__ == '__main__':
    main()
