"""Synthetic /scan for bench-testing the detour planner with the rover up on blocks.

Ray-casts the beam fan against circular obstacles placed in the world frame, using the
navigator's own estimated pose, so the obstacle genuinely approaches as the wheels turn.
Replaces the real lidar: stop sllidar before running this.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

BEAMS = 3240
RANGE_MIN, RANGE_MAX = 0.05, 18.0
LIDAR_OFFSET = 0.34
SCAN_PERIOD = 0.1

# (east, north, radius) in the map frame the navigator surveys in
OBSTACLES = [(12.0, 0.0, 0.30), (26.0, 1.2, 0.50)]


class BenchScan(Node):
    def __init__(self):
        super().__init__('bench_scan')
        self.pose = None
        self.angles = np.linspace(-np.pi, np.pi, BEAMS, endpoint=False)
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.create_subscription(PoseStamped, 'estimated_pose', self.on_pose, 10)
        self.create_timer(SCAN_PERIOD, self.publish_scan)

    def on_pose(self, message):
        q = message.pose.orientation
        self.pose = (message.pose.position.x, message.pose.position.y,
                     2.0 * np.arctan2(q.z, q.w))

    def ranges(self):
        """Closest positive root of |origin + t*direction - centre| = radius, per beam."""
        x, y, yaw = self.pose
        origin = np.array([x + LIDAR_OFFSET * np.cos(yaw), y + LIDAR_OFFSET * np.sin(yaw)])
        world_angles = self.angles + yaw
        direction = np.stack([np.cos(world_angles), np.sin(world_angles)], axis=1)

        best = np.full(BEAMS, np.inf)
        for east, north, radius in OBSTACLES:
            offset = origin - np.array([east, north])
            b = direction @ offset
            c = offset @ offset - radius ** 2
            discriminant = b ** 2 - c
            hit = discriminant > 0.0
            distance = -b[hit] - np.sqrt(discriminant[hit])
            valid = distance > RANGE_MIN
            index = np.flatnonzero(hit)[valid]
            best[index] = np.minimum(best[index], distance[valid])
        return best

    def publish_scan(self):
        if self.pose is None:
            return
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'laser'
        message.angle_min = float(self.angles[0])
        message.angle_max = float(self.angles[-1])
        message.angle_increment = float(self.angles[1] - self.angles[0])
        message.scan_time = SCAN_PERIOD
        message.time_increment = SCAN_PERIOD / BEAMS
        message.range_min, message.range_max = RANGE_MIN, RANGE_MAX
        message.ranges = [float(r) if np.isfinite(r) else float('inf') for r in self.ranges()]
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = BenchScan()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
