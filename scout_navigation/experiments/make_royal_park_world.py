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
OBJECT_CELL = 0.25
OBJECT_HEIGHT = 0.3
TREE_HEIGHT = 3.0
CROWN_WINDOW = 5.0
SMALLEST_OBJECT = 4
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


def objects(east, north, height_above_ground, bounds, ground, plane):
    """Shrubs and tree crowns as closed shells: the measured top, an underside at the object's lowest point.

    Aerial data sees canopies from above only, so a crown is rounded underneath: as deep as its lowest
    rim point at the centre, thinning to the rim; a trunk stands under each crown top, reaching into
    the crown. Low objects reach the ground.
    Returns the shell mesh arrays and the trunks, all in navigator coordinates relative to W00.
    """
    tops, empty = surface(east, north, height_above_ground, bounds, OBJECT_CELL, np.max)
    lows, _ = surface(east, north, height_above_ground, bounds, OBJECT_CELL, np.min)
    occupied = ndimage.binary_closing(~empty, iterations=2) & ~ndimage.binary_erosion(empty, iterations=4)
    labels, count = ndimage.label(occupied, structure=np.ones((3, 3)))
    sizes = ndimage.sum(occupied, labels, range(1, count + 1))
    keep = np.isin(labels, np.flatnonzero(sizes >= SMALLEST_OBJECT) + 1)
    labels[~keep] = 0
    tops = ndimage.gaussian_filter(np.where(keep, tops, 0.0), 1.0) / np.maximum(ndimage.gaussian_filter(keep * 1.0, 1.0), 1e-6)
    base = np.zeros_like(tops)
    trunks = []
    (west, south), _ = bounds
    for label in np.unique(labels[labels > 0]):
        cells = labels == label
        if tops[cells].max() < TREE_HEIGHT:
            continue
        # Rounded crown: full depth down to the lowest rim point at the centre, none at the edge
        lowest = np.percentile(lows[cells], 5)
        inward = ndimage.distance_transform_edt(cells)
        # The outermost ring has no thickness, so a crown needs no side walls
        depth = np.sqrt((inward - 1.0) / max(inward.max() - 1.0, 1.0))
        base[cells] = tops[cells] - (tops[cells] - lowest) * depth[cells]
        peaks = cells & (tops == ndimage.maximum_filter(np.where(cells, tops, 0.0), size=int(CROWN_WINDOW / OBJECT_CELL)))
        for row, column in zip(*np.nonzero(peaks)):
            e, n = np.array([[west + OBJECT_CELL * (column + 0.5)]]), np.array([[south + OBJECT_CELL * (row + 0.5)]])
            local_e, local_n = to_local(e, n, plane)
            crown = np.sqrt(cells.sum() * OBJECT_CELL ** 2 / np.pi / max(len(np.nonzero(peaks)[0]), 1))
            trunks.append((float(local_e[0, 0]), float(local_n[0, 0]), float(ground[row, column]),
                           float(lowest + 0.5 * (tops[row, column] - lowest)), float(np.clip(crown / 12.0, 0.1, 0.4))))
    return keep, tops, base, trunks


def write_shells(path, keep, tops, base, ground, bounds, plane):
    """Top and underside faces per occupied cell, walls where an occupied cell meets an empty one."""
    rows, columns = keep.shape
    (west, south), (east_edge, north_edge) = bounds
    node_e = west + OBJECT_CELL * np.arange(columns + 1)
    node_n = south + OBJECT_CELL * np.arange(rows + 1)
    grid_e, grid_n = np.meshgrid(node_e, node_n)
    local_e, local_n = to_local(grid_e, grid_n, plane)

    def corner(values):
        """Sum over the up to four occupied cells around each grid corner."""
        padded = np.pad(np.where(keep, values, 0.0), 1)
        return sum(padded[dr:dr + rows + 1, dc:dc + columns + 1] for dr in (0, 1) for dc in (0, 1))

    # Corner heights average the occupied cells around each corner
    count = np.maximum(corner(np.ones_like(tops)), 1)
    ground_corner = corner(ground) / count
    top_z, base_z = ground_corner + corner(tops) / count, ground_corner + corner(base) / count
    u = (grid_e - west) / (east_edge - west)
    v = (grid_n - south) / (north_edge - south)
    vertices, faces = [], []

    def vertex(row, column, height):
        vertices.append((local_e[row, column], local_n[row, column], height[row, column], u[row, column], v[row, column]))
        return len(vertices)

    for row, column in zip(*np.nonzero(keep)):
        corners = [(row, column), (row, column + 1), (row + 1, column + 1), (row + 1, column)]
        top = [vertex(r, c, top_z) for r, c in corners]
        bottom = [vertex(r, c, base_z) for r, c in corners]
        faces += [(top[0], top[1], top[2]), (top[0], top[2], top[3]),
                  (bottom[0], bottom[2], bottom[1]), (bottom[0], bottom[3], bottom[2])]
        for side, (dr, dc) in enumerate(((-1, 0), (0, 1), (1, 0), (0, -1))):
            r, c = row + dr, column + dc
            if 0 <= r < rows and 0 <= c < columns and keep[r, c]:
                continue
            a, b = {0: (0, 1), 1: (1, 2), 2: (2, 3), 3: (3, 0)}[side]
            faces += [(bottom[a], bottom[b], top[b]), (bottom[a], top[b], top[a])]
    # Physics needs a normal per vertex: the area-weighted sum of the faces that share it
    points = np.array([vertex[:3] for vertex in vertices])
    corners_of = np.array(faces) - 1
    first = points[corners_of[:, 0]]
    face_normals = np.cross(points[corners_of[:, 1]] - first, points[corners_of[:, 2]] - first)
    normals = np.zeros_like(points)
    for k in range(3):
        np.add.at(normals, corners_of[:, k], face_normals)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
    with open(path, 'w') as obj:
        obj.write(f'mtllib {path.stem}.mtl\nusemtl orthophoto\n')
        obj.writelines(f'v {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z, _, _ in vertices)
        obj.writelines(f'vn {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in normals)
        obj.writelines(f'vt {a:.5f} {b:.5f}\n' for _, _, _, a, b in vertices)
        obj.writelines(f'f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n' for a, b, c in faces)
    return len(faces)


def world(trunks, share):
    trunk_links = ''.join(f'''
      <link name="trunk_{index}">
        <pose>{e:.2f} {n:.2f} {z + top / 2:.2f} 0 0 0</pose>
        <collision name="collision"><geometry><cylinder><radius>{radius:.2f}</radius><length>{top + 0.5:.2f}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{radius:.2f}</radius><length>{top + 0.5:.2f}</length></cylinder></geometry>
          <material><ambient>0.30 0.22 0.15 1</ambient><diffuse>0.36 0.27 0.18 1</diffuse></material></visual>
      </link>''' for index, (e, n, z, top, radius) in enumerate(trunks))
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
      </link>
      <xacro:if value="$(arg trees)">
      <link name="vegetation">
        <collision name="collision">
          <geometry><mesh><uri>file://{share}/worlds/royal_park/objects.obj</uri></mesh></geometry>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>file://{share}/worlds/royal_park/objects.obj</uri></mesh></geometry>
        </visual>
      </link>{trunk_links}
      </xacro:if>
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

    # Objects on the visual grid: heights above the ground model of the same resolution
    dtm, *_ = meshes['terrain']
    cell_e = np.clip(((east - bounds[0][0]) / OBJECT_CELL).astype(int), 0, dtm.shape[1] - 1)
    cell_n = np.clip(((north - bounds[0][1]) / OBJECT_CELL).astype(int), 0, dtm.shape[0] - 1)
    above = height - dtm[cell_n, cell_e]
    standing = (classes == UNCLASSIFIED) & (above > OBJECT_HEIGHT)
    keep, tops, base, trunks = objects(east[standing], north[standing], above[standing], bounds, dtm - origin_height, plane)
    faces = write_shells(OUT / 'objects.obj', keep, tops, base, dtm - origin_height, bounds, plane)
    (OUT / 'objects.mtl').write_text((OUT / 'terrain.mtl').read_text())
    print(f'{keep.sum()} object cells of {OBJECT_CELL} m, {faces} faces, {len(trunks)} trunks')

    # The flat field's world with its ground plane swapped for the survey, and the camera over the grid
    template = TEMPLATE.read_text()
    ground_block = re.search(r'    <model name="ground">.*?</model>\n', template, re.S).group(0)
    text = template.replace(ground_block, world(trunks, '$(arg share)'))
    text = text.replace('<xacro:arg name="longitude" default="144.9614" />',
                        '<xacro:arg name="longitude" default="144.9614" />\n  <xacro:arg name="share" default="" />\n'
                        '  <xacro:arg name="trees" default="false" />')
    text = text.replace('<pose>25 4 45 0 1.5708 1.5708</pose>', '<pose>25 25 75 0 1.5708 1.5708</pose>')
    text = text.replace('<camera_pose>-12 -10 8 0 0.45 0.6</camera_pose>', '<camera_pose>25 -32 24 0 0.55 1.5708</camera_pose>')
    text = text.replace('    <spherical_coordinates>', '    <scene>\n      <ambient>0.6 0.6 0.6 1</ambient>\n      <sky></sky>\n'
                        '    </scene>\n\n    <spherical_coordinates>', 1)
    WORLD.write_text('<?xml version="1.0"?>\n<!-- Generated by experiments/make_royal_park_world.py from the drone survey -->\n'
                     + text.split('\n', 1)[1])
    print(f'wrote {OUT} and {WORLD.name}')


if __name__ == '__main__':
    main()
