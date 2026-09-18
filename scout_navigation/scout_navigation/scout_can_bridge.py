import socket
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

MOTION_COMMAND_ID = 0x111
SYSTEM_STATUS_ID = 0x211
MOTION_FEEDBACK_ID = 0x221
MODE_COMMAND_ID = 0x421

STANDBY_MODE = 0x00
CAN_COMMAND_MODE = 0x01
MODE_NAMES = {0x00: 'standby', 0x01: 'CAN command', 0x02: 'serial', 0x03: 'remote control'}

FRAME_FORMAT = '=IB3x8s'
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)
COMMAND_LIMIT = 1500


class ScoutCanBridge(Node):
    """Translates /cmd_vel into Scout 2.0 motion frames and republishes chassis feedback."""

    def __init__(self):
        super().__init__('scout_can_bridge')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('command_period', 0.02)
        self.declare_parameter('poll_period', 0.005)
        self.declare_parameter('command_timeout', 0.5)

        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.bind((self.get_parameter('can_interface').value,))
        self.socket.setblocking(False)

        self.linear_command = 0
        self.angular_command = 0
        self.control_mode = None
        self.pose = np.zeros(3)
        self.last_feedback_time = self.get_clock().now()
        self.last_command_time = self.get_clock().now()

        self.odometry_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.create_timer(self.get_parameter('command_period').value, self.send_command)
        self.create_timer(self.get_parameter('poll_period').value, self.poll_feedback)

    def on_command(self, message):
        self.linear_command = int(round(message.linear.x * 1000.0))
        self.angular_command = int(round(message.angular.z * 1000.0))
        self.last_command_time = self.get_clock().now()

    def command_expired(self):
        """A lost link (wifi drop while driving from the page) must stop the rover, not hold its speed."""
        silence = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        return silence > self.get_parameter('command_timeout').value

    def send_frame(self, identifier, payload):
        self.socket.send(struct.pack(FRAME_FORMAT, identifier, len(payload), payload))

    def send_command(self):
        """The transmitter outranks CAN, so ask for command mode but never fight it for control."""
        if self.control_mode == STANDBY_MODE:
            self.send_frame(MODE_COMMAND_ID, struct.pack('B', CAN_COMMAND_MODE))
            return
        if self.control_mode != CAN_COMMAND_MODE:
            return

        if self.command_expired():
            self.linear_command = self.angular_command = 0
        payload = struct.pack('>hh4x', max(min(self.linear_command, COMMAND_LIMIT), -COMMAND_LIMIT),
                              max(min(self.angular_command, COMMAND_LIMIT), -COMMAND_LIMIT))
        self.send_frame(MOTION_COMMAND_ID, payload)

    def poll_feedback(self):
        while True:
            frame = self.read_frame()
            if frame is None:
                return
            identifier, payload = frame
            if identifier == SYSTEM_STATUS_ID:
                self.update_mode(payload[1])
            elif identifier == MOTION_FEEDBACK_ID:
                linear, angular = struct.unpack('>hh4x', payload)
                self.publish_odometry(linear / 1000.0, angular / 1000.0)

    def update_mode(self, mode):
        if mode == self.control_mode:
            return
        self.control_mode = mode
        self.get_logger().info(f'chassis control mode: {MODE_NAMES.get(mode, mode)}')

    def publish_odometry(self, speed, yaw_rate):
        """Dead reckons the chassis feedback so the estimator sees measured motion, not commands."""
        now = self.get_clock().now()
        time_step = (now - self.last_feedback_time).nanoseconds * 1e-9
        self.last_feedback_time = now
        self.pose += time_step * np.array([speed * np.cos(self.pose[2]),
                                           speed * np.sin(self.pose[2]), yaw_rate])

        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x, message.pose.pose.position.y = self.pose[:2]
        message.pose.pose.orientation.z = float(np.sin(self.pose[2] / 2.0))
        message.pose.pose.orientation.w = float(np.cos(self.pose[2] / 2.0))
        message.twist.twist.linear.x = speed
        message.twist.twist.angular.z = yaw_rate
        self.odometry_publisher.publish(message)

    def read_frame(self):
        try:
            raw = self.socket.recv(FRAME_SIZE)
        except BlockingIOError:
            return None
        identifier, length, payload = struct.unpack(FRAME_FORMAT, raw)
        return identifier & socket.CAN_EFF_MASK, payload[:length]


def main(args=None):
    rclpy.init(args=args)
    bridge = ScoutCanBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
