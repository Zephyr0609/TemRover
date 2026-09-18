import numpy as np

WGS84_SEMI_MAJOR_AXIS = 6378137.0
WGS84_FLATTENING = 1.0 / 298.257223563
WGS84_ECCENTRICITY_SQUARED = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING)


class LocalTangentPlane:
    """Converts WGS84 coordinates to local ENU metres about a fixed origin."""

    def __init__(self, origin_latitude, origin_longitude):
        self.origin_latitude = origin_latitude
        self.origin_longitude = origin_longitude

        sin_latitude = np.sin(np.radians(origin_latitude))
        curvature = 1.0 - WGS84_ECCENTRICITY_SQUARED * sin_latitude ** 2

        # Prime vertical and meridional radii at the origin
        self.east_scale = np.radians(WGS84_SEMI_MAJOR_AXIS / np.sqrt(curvature)) * np.cos(
            np.radians(origin_latitude))
        self.north_scale = np.radians(
            WGS84_SEMI_MAJOR_AXIS * (1.0 - WGS84_ECCENTRICITY_SQUARED) / curvature ** 1.5)

    def to_local(self, latitude, longitude):
        east = (np.asarray(longitude) - self.origin_longitude) * self.east_scale
        north = (np.asarray(latitude) - self.origin_latitude) * self.north_scale
        return np.stack([east, north], axis=-1)

    def to_geodetic(self, local_position):
        local_position = np.asarray(local_position)
        longitude = self.origin_longitude + local_position[..., 0] / self.east_scale
        latitude = self.origin_latitude + local_position[..., 1] / self.north_scale
        return latitude, longitude


def wrap_to_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi
