"""Writes the overhead camera stream to a video file so a run can be shown without the simulator."""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

FRAME_RATE = 25.0


class VideoRecorder(Node):
    """Encodes every frame the overhead camera publishes."""

    def __init__(self, filename):
        super().__init__('video_recorder')
        self.writer = None
        self.filename = filename
        self.frames = 0
        self.create_subscription(Image, 'survey_camera', self.on_image, qos_profile_sensor_data)

    def on_image(self, message):
        frame = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
        if self.writer is None:
            self.writer = cv2.VideoWriter(self.filename, cv2.VideoWriter_fourcc(*'mp4v'),
                                          FRAME_RATE, (message.width, message.height))
            print(f'recording {message.width}x{message.height} to {self.filename}')
        self.writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        self.frames += 1


def main():
    seconds, filename = float(sys.argv[1]), sys.argv[2]
    rclpy.init()
    recorder = VideoRecorder(filename)

    start = recorder.get_clock().now()
    while (recorder.get_clock().now() - start).nanoseconds < seconds * 1e9:
        rclpy.spin_once(recorder, timeout_sec=0.05)

    if recorder.writer is not None:
        recorder.writer.release()
    print(f'wrote {recorder.frames} frames, {recorder.frames / FRAME_RATE:.1f} s of video')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
