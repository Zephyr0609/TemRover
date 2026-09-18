"""Checks the CAN bridge against the Scout 2.0 motion frames over a virtual interface."""
import socket
import struct

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from scout_navigation.scout_can_bridge import (CAN_COMMAND_MODE, FRAME_FORMAT, FRAME_SIZE,
                                               MOTION_COMMAND_ID, MOTION_FEEDBACK_ID,
                                               SYSTEM_STATUS_ID, ScoutCanBridge)

INTERFACE = 'vcan0'
COMMAND_SPEED = (1.0, 0.5)
FEEDBACK_SPEED = (-0.5, -1.2)


class Probe(Node):
    """Drives the bridge from the ROS side and records what comes back."""

    def __init__(self):
        super().__init__('can_probe')
        self.odometry = []
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odometry, 10)

    def on_odometry(self, message):
        self.odometry.append((message.twist.twist.linear.x, message.twist.twist.angular.z))

    def send(self, linear, angular):
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        self.publisher.publish(command)


def open_monitor():
    monitor = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    monitor.bind((INTERFACE,))
    monitor.setblocking(False)
    return monitor


def drain(monitor, identifier):
    payloads = []
    while True:
        try:
            raw = monitor.recv(FRAME_SIZE)
        except BlockingIOError:
            return payloads
        frame, length, payload = struct.unpack(FRAME_FORMAT, raw)
        if frame == identifier:
            payloads.append(payload[:length])


def main():
    rclpy.init(args=['--ros-args', '-p', f'can_interface:={INTERFACE}'])
    bridge = ScoutCanBridge()
    probe = Probe()
    monitor = open_monitor()

    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(probe)

    monitor.send(struct.pack(FRAME_FORMAT, SYSTEM_STATUS_ID, 8,
                             bytes([0x00, CAN_COMMAND_MODE, 0x01, 0x02, 0, 0, 0, 0])))
    probe.send(*COMMAND_SPEED)
    for _ in range(400):
        executor.spin_once(timeout_sec=0.01)
    sent = drain(monitor, MOTION_COMMAND_ID)

    monitor.send(struct.pack(FRAME_FORMAT, MOTION_FEEDBACK_ID, 8,
                             struct.pack('>hh4x', int(FEEDBACK_SPEED[0] * 1000),
                                         int(FEEDBACK_SPEED[1] * 1000))))
    for _ in range(200):
        executor.spin_once(timeout_sec=0.01)

    expected = struct.pack('>hh4x', int(COMMAND_SPEED[0] * 1000), int(COMMAND_SPEED[1] * 1000))
    print(f'command frames on {INTERFACE}: {len(sent)}')
    print(f'  payload   {sent[-1].hex().upper() if sent else "none"}')
    print(f'  expected  {expected.hex().upper()}')
    print(f'  match     {bool(sent) and sent[-1] == expected}')
    print(f'feedback decoded into /odom: {probe.odometry[-1] if probe.odometry else "none"}')
    print(f'  expected  {FEEDBACK_SPEED}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
