import numpy as np

from .geodesy import wrap_to_pi


class DifferentialDriveRover:
    """Unicycle model with the Scout 2.0 command limits and first-order actuator response."""

    def __init__(self, max_linear_speed, max_angular_speed, max_linear_acceleration,
                 max_angular_acceleration, yaw_slip_factor):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration
        self.yaw_slip_factor = yaw_slip_factor

        self.position = np.zeros(2)
        self.heading = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0

    def step(self, linear_command, angular_command, time_step):
        linear_target = np.clip(linear_command, -self.max_linear_speed, self.max_linear_speed)
        angular_target = np.clip(angular_command, -self.max_angular_speed, self.max_angular_speed)

        linear_change = self.max_linear_acceleration * time_step
        angular_change = self.max_angular_acceleration * time_step
        self.linear_speed += np.clip(linear_target - self.linear_speed, -linear_change, linear_change)
        self.angular_speed += np.clip(angular_target - self.angular_speed, -angular_change, angular_change)

        # Skid steering loses part of the commanded yaw rate to wheel scrub
        yaw_rate = self.angular_speed * (1.0 - self.yaw_slip_factor)
        self.position += self.linear_speed * time_step * np.array(
            [np.cos(self.heading), np.sin(self.heading)])
        self.heading = wrap_to_pi(self.heading + yaw_rate * time_step)
