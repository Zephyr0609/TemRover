"""MGA2020 grid coordinates to geographic, and the survey line they describe.

The navigator is configured in latitude, longitude and a bearing measured in the local
tangent plane, but the site is surveyed in MGA zone 55 eastings and northings. This is the
Krueger inverse that bridges them, checked against resource/SOUTHLAWN2020.txt and the
geographic copy beside it.
"""
import sys

import numpy as np

from scout_navigation.geodesy import LocalTangentPlane

GRS80_SEMI_MAJOR_AXIS = 6378137.0
GRS80_FLATTENING = 1.0 / 298.257222101
CENTRAL_SCALE = 0.9996
FALSE_EASTING = 500000.0
FALSE_NORTHING = 10000000.0

# Krueger series, inverse direction; four terms hold millimetres over a zone
BETA = (lambda n: (n / 2 - 2 * n**2 / 3 + 37 * n**3 / 96 - n**4 / 360,
                   n**2 / 48 + n**3 / 15 - 437 * n**4 / 1440,
                   17 * n**3 / 480 - 37 * n**4 / 840,
                   4397 * n**4 / 161280))
DELTA = (lambda n: (2 * n - 2 * n**2 / 3 - 2 * n**3 + 116 * n**4 / 45,
                    7 * n**2 / 3 - 8 * n**3 / 5 - 227 * n**4 / 45,
                    56 * n**3 / 15 - 136 * n**4 / 35,
                    4279 * n**4 / 630))


def central_meridian(zone):
    return -183.0 + 6.0 * zone


def to_geographic(easting, northing, zone=55):
    """Geodetic latitude and longitude, in degrees, of an MGA grid coordinate."""
    n = GRS80_FLATTENING / (2.0 - GRS80_FLATTENING)
    rectifying_radius = (GRS80_SEMI_MAJOR_AXIS / (1.0 + n)
                         * (1.0 + n**2 / 4.0 + n**4 / 64.0))

    xi = (np.asarray(northing, dtype=float) - FALSE_NORTHING) / (CENTRAL_SCALE * rectifying_radius)
    eta = (np.asarray(easting, dtype=float) - FALSE_EASTING) / (CENTRAL_SCALE * rectifying_radius)

    conformal_xi, conformal_eta = xi, eta
    for index, beta in enumerate(BETA(n), start=1):
        conformal_xi = conformal_xi - beta * np.sin(2 * index * xi) * np.cosh(2 * index * eta)
        conformal_eta = conformal_eta - beta * np.cos(2 * index * xi) * np.sinh(2 * index * eta)

    conformal_latitude = np.arcsin(np.sin(conformal_xi) / np.cosh(conformal_eta))
    longitude = np.radians(central_meridian(zone)) + np.arctan(
        np.sinh(conformal_eta) / np.cos(conformal_xi))

    latitude = conformal_latitude
    for index, delta in enumerate(DELTA(n), start=1):
        latitude = latitude + delta * np.sin(2 * index * conformal_latitude)
    return np.degrees(latitude), np.degrees(longitude)


def survey_line(start, end, zone=55):
    """Origin, bearing and length the navigator needs, from the two surveyed corners.

    The bearing comes out of the navigator's own tangent plane rather than from a grid
    convergence formula, so it is the angle the navigator will itself compute.
    """
    start_latitude, start_longitude = to_geographic(*start, zone=zone)
    end_latitude, end_longitude = to_geographic(*end, zone=zone)
    plane = LocalTangentPlane(float(start_latitude), float(start_longitude))
    east, north = plane.to_local(float(end_latitude), float(end_longitude))
    return ((float(start_latitude), float(start_longitude)),
            (float(end_latitude), float(end_longitude)),
            float(np.arctan2(north, east)), float(np.hypot(east, north)))


def validate():
    """Checks the inverse against the surveyed pair already in resource/."""
    import csv
    import pathlib
    resource = pathlib.Path(__file__).resolve().parent.parent / 'resource'
    grid = {row[0]: (float(row[1]), float(row[2]))
            for row in csv.reader((resource / 'SOUTHLAWN2020.txt').read_text().splitlines()[1:])
            if len(row) > 2}
    reference = {row[0]: (float(row[1]), float(row[2]))
                 for row in csv.reader((resource / 'south_lawn_geographic.csv').read_text().splitlines()[1:])
                 if len(row) > 2}

    worst = 0.0
    for name, (easting, northing) in grid.items():
        if name not in reference:
            continue
        latitude, longitude = to_geographic(easting, northing)
        plane = LocalTangentPlane(*reference[name])
        offset = plane.to_local(float(latitude), float(longitude))
        worst = max(worst, float(np.hypot(*offset)))
        print(f'  {name}  {latitude:.8f} {longitude:.8f}   差 {np.hypot(*offset) * 1000:6.2f} mm')
    print(f'\n  最大偏差 {worst * 1000:.2f} mm')


def main():
    if sys.argv[1:2] == ['--validate']:
        validate()
        return
    start = (float(sys.argv[1]), float(sys.argv[2]))
    end = (float(sys.argv[3]), float(sys.argv[4]))
    (start_geographic, end_geographic, bearing, length) = survey_line(start, end)
    print(f'起点  E {start[0]:.3f}  N {start[1]:.3f}  ->  '
          f'{start_geographic[0]:.8f}, {start_geographic[1]:.8f}')
    print(f'终点  E {end[0]:.3f}  N {end[1]:.3f}  ->  '
          f'{end_geographic[0]:.8f}, {end_geographic[1]:.8f}')
    print(f'\n测线长度 {length:.3f} m')
    print(f'方位角   {bearing:.6f} rad  ({np.degrees(bearing):+.3f} deg，从正东逆时针)')
    print(f'\n    origin_latitude: {start_geographic[0]:.8f}')
    print(f'    origin_longitude: {start_geographic[1]:.8f}')
    print(f'    grid_bearing: {bearing:.6f}')
    print(f'    grid_length: {length:.2f}')


if __name__ == '__main__':
    main()
