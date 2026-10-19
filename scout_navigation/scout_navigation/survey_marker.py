"""The survey lines as a static Gazebo model: a ribbon per line and a disc at each surveyed end.

Everything lies flat on the ground: the simulated lidar sees visuals, and anything taller is an obstacle.
"""
import numpy as np

RIBBON_WIDTH = 0.12
RIBBON_HEIGHT = 0.02
DISC_RADIUS = 0.35
START_COLOUR = '0.15 0.65 0.25'
END_COLOUR = '0.85 0.30 0.10'


def material(colour):
    return (f'<material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse>'
            f'<emissive>{colour} 1</emissive></material>')


def ribbon(name, start, end):
    centre, offset = (start + end) / 2.0, end - start
    heading = np.arctan2(offset[1], offset[0])
    return f'''    <link name="{name}">
      <pose>{centre[0]:.3f} {centre[1]:.3f} {RIBBON_HEIGHT / 2:.3f} 0 0 {heading:.4f}</pose>
      <visual name="visual">
        <geometry><box><size>{np.hypot(*offset):.3f} {RIBBON_WIDTH} {RIBBON_HEIGHT}</size></box></geometry>
        {material('0.95 0.95 0.95')}
      </visual>
    </link>
'''


def disc(name, point, colour):
    return f'''    <link name="{name}">
      <pose>{point[0]:.3f} {point[1]:.3f} {RIBBON_HEIGHT / 2:.3f} 0 0 0</pose>
      <visual name="visual">
        <geometry><cylinder><radius>{DISC_RADIUS}</radius><length>{RIBBON_HEIGHT}</length></cylinder></geometry>
        {material(colour)}
      </visual>
    </link>
'''


def survey_marker_sdf(lines):
    """Lines as (start, end) map points; the green disc is where a line starts, the orange where it ends."""
    links = []
    for index, (start, end) in enumerate(lines):
        links += [ribbon(f'line_{index}', start, end),
                  disc(f'line_{index}_start', start, START_COLOUR),
                  disc(f'line_{index}_end', end, END_COLOUR)]
    return ('<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="survey_lines">\n'
            '    <static>true</static>\n' + ''.join(links) + '  </model>\n</sdf>\n')
