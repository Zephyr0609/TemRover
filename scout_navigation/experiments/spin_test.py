"""Measures the yaw rate the rover actually achieves in place, binned by heading."""
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

COMMANDED_RATE = 1.0
SAMPLE_COUNT = 1200
BIN_COUNT = 8


class SpinTest(Node):
    """Drives a constant in-place rotation and records what the body does."""

    def __init__(self):
        super().__init__('spin_test')
        self.samples = []
        self.create_subscription(Imu, 'imu/data', self.on_imu, 50)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def on_imu(self, message):
        heading = 2.0 * np.arctan2(message.orientation.z, message.orientation.w)
        self.samples.append((heading, message.angular_velocity.z))

    def run(self):
        command = Twist()
        command.angular.z = COMMANDED_RATE
        for _ in range(SAMPLE_COUNT):
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publisher.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    test = SpinTest()
    test.run()

    samples = np.array(test.samples)
    edges = np.linspace(-np.pi, np.pi, BIN_COUNT + 1)
    print(f'commanded {COMMANDED_RATE} rad/s, {len(samples)} samples')
    for low, high in zip(edges[:-1], edges[1:]):
        inside = (samples[:, 0] >= low) & (samples[:, 0] < high)
        if inside.sum() < 5:
            continue
        print(f'  heading {np.degrees(low):+5.0f}..{np.degrees(high):+5.0f} deg  '
              f'achieved {samples[inside, 1].mean():+.3f} rad/s  ({int(inside.sum())} samples)')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
