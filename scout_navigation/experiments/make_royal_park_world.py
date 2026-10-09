"""Builds the Royal Park Gazebo world from the drone survey: textured ground mesh, collision mesh, trees.

Input is the survey folder (default ../royal_park next to this package): the classified LAZ point
cloud (GDA94) and the GDA2020 orthophoto. The cloud is moved to GDA2020 to match the surveyed grid,
then every vertex goes MGA2020 -> latitude/longitude -> the navigator's own tangent plane at W00,
so the terrain sits exactly where the simulated GNSS puts the rover.

Ground: median of the ground-class points per cell, holes under canopies filled from the nearest
cell. Trees: aerial points see only canopies, so each canopy top becomes a trunk the lidar can hit
and a crown that is drawn but not collided with.
"""
import pathlib
import re
import sys

import laspy
import numpy as np
import rasterio
from PIL import Image
from pyproj import Transformer
from rasterio.windows import from_bounds
from scipy import ndimage
from scout_navigation.geodesy import LocalTangentPlane

SURVEY = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[2] / 'royal_park')
CLOUD = SURVEY / 'a_From_Drone_Survey' / 'EBTA_Survey_1st_July_2026_Royal_Park_LAZ_GDA94_MGA_zone_55_22_495_956_points.laz'
ORTHO = SURVEY / 'b_Grid_Coordinates' / 'Survey_01_07_2026_Royal_Park_GeoTIFF_GDA2020_MGA_zone55.tif'
GRID = SURVEY / 'b_Grid_Coordinates' / 'RoyalPark_Grid_Coordinates.csv'
OUT = pathlib.Path(__file__).resolve().parents[1] / 'worlds' / 'royal_park'

# Margins around the 50 m grid: lines run past both ends by a headland and a U-turn bulb
MARGIN_ALONG, MARGIN_ACROSS = 30.0, 15.0
VISUAL_CELL, COLLISION_CELL = 0.25, 0.5
TEXTURE_WIDTH = 4096
CANOPY_HEIGHT = 2.0
CROWN_WINDOW = 5.0
TRUNK_RADIUS, TRUNK_HEIGHT = 0.2, 2.5
TEMPLATE = OUT.parent / 'flat_field.sdf.xacro'
WORLD = OUT.parent / 'royal_park.sdf.xacro'

GROUND, UNCLASSIFIED = 2, 0


def grid_points():
    rows = np.genfromtxt(GRID, delimiter=',', names=True, dtype=None, encoding='utf-8')
    return {row['PointID']: np.array([row['Easting'], row['Northing']]) for row in rows}


def surface(east, north, values, bounds, cell, statistic):
    """Per-cell statistic on a regular MGA grid, holes filled from the nearest observed cell."""
    (west, south), (east_edge, north_edge) = bounds
    columns, rows = int(round((east_edge - west) / cell)), int(round((north_edge - south) / cell))
    index_e = np.clip(((east - west) / cell).astype(int), 0, columns - 1)
    index_n = np.clip(((north - south) / cell).astype(int), 0, rows - 1)
    flat = index_n * columns + index_e
    result = np.full(rows * columns, np.nan)
    order = np.argsort(flat)
    flat, ordered = flat[order], values[order]
    starts = np.flatnonzero(np.r_[True, np.diff(flat) > 0])
    for start, stop in zip(starts, np.r_[starts[1:], len(flat)]):
        result[flat[start]] = statistic(ordered[start:stop])
    result = result.reshape(rows, columns)
    missing = np.isnan(result)
    nearest = ndimage.distance_transform_edt(missing, return_distances=False, return_indices=True)
    return result[tuple(nearest)], missing


def node_coordinates(bounds, cell):
    (west, south), (east_edge, north_edge) = bounds
    east = west + cell * (np.arange(int(round((east_edge - west) / cell))) + 0.5)
    north = south + cell * (np.arange(int(round((north_edge - south) / cell))) + 0.5)
    return np.meshgrid(east, north)


def to_local(east, north, plane):
    """MGA2020 grid coordinates to the navigator's east/north metres about W00."""
    longitude, latitude = Transformer.from_crs(7855, 7844, always_xy=True).transform(east.ravel(), north.ravel())
    local = np.array([plane.to_local(lat, lon) for lat, lon in zip(np.atleast_1d(latitude), np.atleast_1d(longitude))])
    return local[:, 0].reshape(east.shape), local[:, 1].reshape(east.shape)


def vertex_normals(local_e, local_n, height):
    """Unit normals from the surface slope; physics needs one per vertex."""
    rows, columns = height.shape
    de_dc, dn_dc, dz_dc = (np.gradient(a, axis=1) for a in (local_e, local_n, height))
    de_dr, dn_dr, dz_dr = (np.gradient(a, axis=0) for a in (local_e, local_n, height))
    normal = np.cross(np.stack([de_dc, dn_dc, dz_dc], -1), np.stack([de_dr, dn_dr, dz_dr], -1))
    return (normal / np.linalg.norm(normal, axis=-1, keepdims=True)).reshape(rows * columns, 3)


def write_obj(path, local_e, local_n, height, uv, material):
    """Regular grid of vertices, two triangles per cell, texture coordinates from MGA position."""
    rows, columns = height.shape
    with open(path, 'w') as obj:
        if material:
            obj.write(f'mtllib {path.stem}.mtl\nusemtl {material}\n')
        for e, n, z in zip(local_e.ravel(), local_n.ravel(), height.ravel()):
            obj.write(f'v {e:.3f} {n:.3f} {z:.3f}\n')
        for x, y, z in vertex_normals(local_e, local_n, height):
            obj.write(f'vn {x:.4f} {y:.4f} {z:.4f}\n')
        if uv is not None:
            for u, v in zip(*uv):
                obj.write(f'vt {u:.5f} {v:.5f}\n')
        index = np.arange(rows * columns).reshape(rows, columns) + 1
        a, b, c, d = index[:-1, :-1], index[:-1, 1:], index[1:, 1:], index[1:, :-1]
        for face in np.column_stack([a.ravel(), b.ravel(), c.ravel(), a.ravel(), c.ravel(), d.ravel()]).reshape(-1, 3):
            obj.write('f ' + ' '.join(f'{i}/{i}/{i}' if uv is not None else f'{i}//{i}' for i in face) + '\n')


def trees(east, north, height_above_ground, bounds, ground, plane):
    """Crown tops are local maxima of the canopy height model; one trunk under each, standing on the ground."""
    canopy, empty = surface(east, north, height_above_ground, bounds, 0.5, np.max)
    canopy[empty | (canopy < CANOPY_HEIGHT)] = 0.0
    peaks = (canopy == ndimage.maximum_filter(canopy, size=int(CROWN_WINDOW / 0.5))) & (canopy > CANOPY_HEIGHT)
    labels, _ = ndimage.label(canopy > CANOPY_HEIGHT)
    rows, columns = np.nonzero(peaks)
    (west, south), _ = bounds
    found = []
    for row, column in zip(rows, columns):
        crown_cells = np.count_nonzero(labels == labels[row, column])
        e, n = np.array([[west + 0.5 * (column + 0.5)]]), np.array([[south + 0.5 * (row + 0.5)]])
        local_e, local_n = to_local(e, n, plane)
        found.append((float(local_e[0, 0]), float(local_n[0, 0]), float(ground[row, column]), float(canopy[row, column]),
                      min(np.sqrt(crown_cells * 0.25 / np.pi), CROWN_WINDOW)))
    return found


def world(trees_found, share):
    tree_links = ''.join(f'''
      <link name="tree_{index}">
        <pose>{e:.2f} {n:.2f} {z:.2f} 0 0 0</pose>
        <collision name="trunk"><pose>0 0 {TRUNK_HEIGHT / 2 - 1.0:.2f} 0 0 0</pose>
          <geometry><cylinder><radius>{TRUNK_RADIUS}</radius><length>{TRUNK_HEIGHT + 2.0}</length></cylinder></geometry>
        </collision>
        <visual name="trunk"><pose>0 0 {TRUNK_HEIGHT / 2 - 1.0:.2f} 0 0 0</pose>
          <geometry><cylinder><radius>{TRUNK_RADIUS}</radius><length>{TRUNK_HEIGHT + 2.0}</length></cylinder></geometry>
          <material><ambient>0.35 0.25 0.15 1</ambient><diffuse>0.35 0.25 0.15 1</diffuse></material></visual>
        <visual name="crown"><pose>0 0 {top - crown:.2f} 0 0 0</pose>
          <geometry><sphere><radius>{crown:.2f}</radius></sphere></geometry>
          <material><ambient>0.2 0.35 0.15 0.6</ambient><diffuse>0.2 0.35 0.15 0.6</diffuse></material></visual>
      </link>''' for index, (e, n, z, top, crown) in enumerate(trees_found))
    return f'''
    <model name="royal_park_terrain">
      <static>true</static>
      <link name="ground">
        <collision name="collision">
          <geometry><mesh><uri>file://{share}/worlds/royal_park/collision.obj</uri></mesh></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>0.7</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>file://{share}/worlds/royal_park/terrain.obj</uri></mesh></geometry>
        </visual>
      </link>{tree_links}
    </model>
'''


def main():
    grid = grid_points()
    origin = grid['W00']
    longitude, latitude = Transformer.from_crs(7855, 7844, always_xy=True).transform(origin[0], origin[1])
    plane = LocalTangentPlane(latitude, longitude)
    east_end = to_local(np.array([grid['E00'][0]]), np.array([grid['E00'][1]]), plane)
    bearing = float(np.arctan2(east_end[1][0], east_end[0][0]))
    corners = np.array(list(grid.values()))
    bounds = np.array([corners.min(axis=0) - [MARGIN_ALONG, MARGIN_ACROSS], corners.max(axis=0) + [MARGIN_ALONG, MARGIN_ACROSS]])
    print(f'W00 at {latitude:.8f}, {longitude:.8f}; line bearing {bearing:+.5f} rad; crop {np.ptp(bounds, axis=0).round(1)} m')

    cloud = laspy.read(CLOUD)
    east, north = Transformer.from_crs(28355, 7855, always_xy=True).transform(np.asarray(cloud.x), np.asarray(cloud.y))
    height, classes = np.asarray(cloud.z), np.asarray(cloud.classification)
    inside = (east > bounds[0][0]) & (east < bounds[1][0]) & (north > bounds[0][1]) & (north < bounds[1][1])
    east, north, height, classes = east[inside], north[inside], height[inside], classes[inside]
    ground = classes == GROUND
    print(f'{inside.sum()} points in the crop, {ground.sum()} ground')

    OUT.mkdir(parents=True, exist_ok=True)
    meshes = {}
    for name, cell in (('terrain', VISUAL_CELL), ('collision', COLLISION_CELL)):
        dtm, missing = surface(east[ground], north[ground], height[ground], bounds, cell, np.median)
        node_e, node_n = node_coordinates(bounds, cell)
        meshes[name] = (dtm, node_e, node_n)
        print(f'{name}: {dtm.shape[1]} x {dtm.shape[0]} cells of {cell} m, {missing.mean() * 100:.1f} % filled from neighbours')
    dtm, *_ = meshes['collision']
    origin_height = float(ndimage.map_coordinates(dtm, [[(origin[1] - bounds[0][1]) / COLLISION_CELL - 0.5],
                                                        [(origin[0] - bounds[0][0]) / COLLISION_CELL - 0.5]], order=1)[0])
    print(f'ground at W00 {origin_height:.2f} m (survey peg 36.00 m)')

    for name, (dtm, node_e, node_n) in meshes.items():
        local_e, local_n = to_local(node_e, node_n, plane)
        uv = None
        if name == 'terrain':
            uv = ((node_e.ravel() - bounds[0][0]) / np.ptp(bounds[:, 0]), (node_n.ravel() - bounds[0][1]) / np.ptp(bounds[:, 1]))
        write_obj(OUT / f'{name}.obj', local_e, local_n, dtm - origin_height, uv, 'orthophoto' if name == 'terrain' else None)

    with rasterio.open(ORTHO) as ortho:
        window = from_bounds(bounds[0][0], bounds[0][1], bounds[1][0], bounds[1][1], ortho.transform)
        rows = int(TEXTURE_WIDTH * np.ptp(bounds[:, 1]) / np.ptp(bounds[:, 0]))
        image = ortho.read([1, 2, 3], window=window, out_shape=(3, rows, TEXTURE_WIDTH))
    Image.fromarray(np.moveaxis(image, 0, -1)).save(OUT / 'terrain.jpg', quality=90)
    (OUT / 'terrain.mtl').write_text('newmtl orthophoto\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd terrain.jpg\n')

    dtm, *_ = meshes['collision']
    cell_e = np.clip(((east - bounds[0][0]) / COLLISION_CELL).astype(int), 0, dtm.shape[1] - 1)
    cell_n = np.clip(((north - bounds[0][1]) / COLLISION_CELL).astype(int), 0, dtm.shape[0] - 1)
    above = height - dtm[cell_n, cell_e]
    canopy = classes == UNCLASSIFIED
    found = trees(east[canopy], north[canopy], above[canopy], bounds, dtm - origin_height, plane)

    # The flat field's world with its ground plane swapped for the survey, and the camera over the grid
    template = TEMPLATE.read_text()
    ground_block = re.search(r'    <model name="ground">.*?</model>\n', template, re.S).group(0)
    text = template.replace(ground_block, world(found, '$(arg share)'))
    text = text.replace('<xacro:arg name="longitude" default="144.9614" />',
                        '<xacro:arg name="longitude" default="144.9614" />\n  <xacro:arg name="share" default="" />')
    text = text.replace('<pose>25 4 45 0 1.5708 1.5708</pose>', '<pose>25 25 75 0 1.5708 1.5708</pose>')
    WORLD.write_text('<?xml version="1.0"?>\n<!-- Generated by experiments/make_royal_park_world.py from the drone survey -->\n'
                     + text.split('\n', 1)[1])
    print(f'{len(found)} trees; wrote {OUT} and {WORLD.name}')


if __name__ == '__main__':
    main()
