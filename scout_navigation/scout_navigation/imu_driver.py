"""ICM-20948 on the LattePanda I2C bus, published as imu/data for the navigator.

Adapted from ej5962/Capstone imu_processing_raw.py: same sensor reads and unit conversions, with
timestamps, a frame id, and the gyro bias measured while the rover stands still at power-up.
"""
import numpy as np
import rclpy
from icm20948 import ICM20948
from rclpy.node import Node
from sensor_msgs.msg import Imu

GRAVITY = 9.80665


class ImuDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        self.declare_parameter('imu_rate', 100.0)
        self.declare_parameter('imu_bias_duration', 3.0)
        # +1 when the chip's z axis points up; -1 if it is mounted upside down
        self.declare_parameter('imu_yaw_sign', 1.0)
        self.settings = {name: self.get_parameter(name).value
                         for name in ('imu_rate', 'imu_bias_duration', 'imu_yaw_sign')}

        # The Pimoroni library opens I2C bus 1, where the sensor sits on the LattePanda
        self.imu = ICM20948()
        self.bias_samples = []
        self.gyro_bias = None
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.create_timer(1.0 / self.settings['imu_rate'], self.publish_imu)
        self.get_logger().info(f"Hold still: measuring gyro bias for {self.settings['imu_bias_duration']:.0f} s")

    def publish_imu(self):
        accel, gyro = self.imu.read_accelerometer_gyro_data()
        gyro = np.radians(gyro)

        # The rover is idle at power-up, so the first seconds of gyro output are pure bias
        if self.gyro_bias is None:
            self.bias_samples.append(gyro)
            if len(self.bias_samples) < self.settings['imu_bias_duration'] * self.settings['imu_rate']:
                return
            self.gyro_bias = np.mean(self.bias_samples, axis=0)
            self.get_logger().info(f'Gyro bias {np.degrees(self.gyro_bias).round(3)} deg/s, publishing imu/data')
        gyro = gyro - self.gyro_bias

        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'imu_link'
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = (
            float(value) * GRAVITY for value in accel)
        message.angular_velocity.x, message.angular_velocity.y = float(gyro[0]), float(gyro[1])
        message.angular_velocity.z = float(gyro[2]) * self.settings['imu_yaw_sign']
        # No orientation estimate: identity quaternion means level to the navigator's ground filter
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1.0
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    driver = ImuDriver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
