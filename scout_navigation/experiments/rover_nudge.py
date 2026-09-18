"""Drives the rover in a straight line at survey speed and compares commanded against measured motion."""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from scout_navigation.scout_can_bridge import ScoutCanBridge

MODE_LABEL = {0x00: 'standby', 0x01: 'CAN command', 0x02: 'serial', 0x03: 'remote control'}


class StraightRun(Node):
    """Publishes survey cruise speed until the chassis reports the target distance."""

    def __init__(self):
        super().__init__('straight_run')
        self.declare_parameter('cruise_speed', 1.5)
        self.declare_parameter('test_distance', 2.0)
        self.declare_parameter('settle_duration', 1.5)

        self.speed = self.get_parameter('cruise_speed').value
        self.distance = self.get_parameter('test_distance').value
        self.travel = 0.0
        self.samples = []

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odometry, 10)

    def on_odometry(self, message):
        self.travel = message.pose.pose.position.x
        self.samples.append(message.twist.twist.linear.x)

    def send(self, speed):
        command = Twist()
        command.linear.x = speed
        self.publisher.publish(command)


def spin_for(executor, duration):
    """Spins freely so the bridge timers are never starved by the caller."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.001)


def spin_until_travelled(executor, run, timeout):
    """Distance bounds the run and the timeout bounds it again if feedback never arrives."""
    start = run.travel
    deadline = time.monotonic() + timeout
    while run.travel - start < run.distance and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.001)
    return run.travel - start


def main():
    rclpy.init()
    bridge = ScoutCanBridge()
    run = StraightRun()

    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(run)

    spin_for(executor, 1.0)
    print(f'control mode: {MODE_LABEL.get(bridge.control_mode, bridge.control_mode)}')
    baseline = len(run.samples)

    timeout = 3.0 * run.distance / run.speed
    run.send(run.speed)
    travelled = spin_until_travelled(executor, run, timeout)
    run.send(0.0)
    spin_for(executor, run.get_parameter('settle_duration').value)

    speeds = np.array(run.samples[baseline:])
    print(f'commanded {run.speed:.2f} m/s until {run.distance:.1f} m travelled')
    print(f'  feedback samples {len(speeds)}')
    print(f'  measured speed  mean {speeds.mean():+.3f}  peak {speeds.max():+.3f} m/s')
    print(f'  dead-reckoned travel {travelled:+.3f} m')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
