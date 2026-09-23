import numpy as np


def scan_points(scan, attitude, sensor_height, minimum_obstacle_height, step_threshold,
                maximum_object_angle, lidar_yaw, field_of_view):
    """Body-frame x, y of every beam that hit a standing object rather than the ground.

    lidar_yaw is where the scan's own zero bearing points, measured from the rover's
    forward direction, so a unit bolted on facing aft is pi rather than a code change.
    Only beams within field_of_view centred on the rover's heading are kept.
    """
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges)) + lidar_yaw
    roll, pitch = attitude

    # Vertical component of each beam once the rover is tilted
    elevation = -np.sin(pitch) * np.cos(angles) + np.cos(pitch) * np.sin(roll) * np.sin(angles)
    endpoint_height = sensor_height + ranges * elevation
    valid = ((ranges > scan.range_min) & (ranges < scan.range_max)
             & (endpoint_height > minimum_obstacle_height))

    ranges, angles = standing_objects(ranges[valid], angles[valid], step_threshold,
                                      maximum_object_angle)
    # Segment on the full scan first so an object straddling the cone edge keeps its true width
    ahead = np.abs(np.angle(np.exp(1j * angles))) <= field_of_view / 2.0
    ranges, angles = ranges[ahead], angles[ahead]
    return np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles)])


def standing_objects(ranges, angles, step_threshold, maximum_object_angle):
    """Keeps compact segments — sloping terrain spans the scan, a standing object does not."""
    if len(ranges) < 2:
        return ranges, angles

    segment = np.concatenate([[0], np.cumsum(np.abs(np.diff(ranges)) > step_threshold)])
    compact = np.zeros(len(ranges), dtype=bool)
    for index in range(segment[-1] + 1):
        members = segment == index
        if np.ptp(angles[members]) < maximum_object_angle:
            compact |= members
    return ranges[compact], angles[compact]



def to_world(points, position, heading, sensor_offset):
    """Places sensor-frame points in the world from the rover pose and the sensor's forward offset."""
    rotation = np.array([[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]])
    return position + (points + np.array([sensor_offset, 0.0])) @ rotation.T


def line_frame(points, start, end):
    """Along-track and signed cross-track coordinates of points relative to the segment start->end."""
    direction = (end - start) / np.hypot(*(end - start))
    normal = np.array([-direction[1], direction[0]])
    offsets = points - start
    return offsets @ direction, offsets @ normal
