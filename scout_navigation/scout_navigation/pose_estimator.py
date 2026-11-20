"""Pose estimator: an extended Kalman filter for the rover's position and heading.

Predicts with wheel speed and yaw rate; corrects with GNSS position and GNSS direction of travel.
"""
import numpy as np

from .geodesy import wrap_to_pi


class PoseEstimator:
    """State is [east, north, heading] of the rover centre (base_link), in metres and radians."""

    def __init__(self, position_process_noise, yaw_process_noise, gnss_noise, course_noise,
                 minimum_course_speed, antenna_offset, initial_heading_noise):
        self.process_noise = np.diag([position_process_noise, position_process_noise,
                                      yaw_process_noise])
        self.gnss_noise = gnss_noise ** 2 * np.eye(2)
        self.course_noise = course_noise ** 2
        self.minimum_course_speed = minimum_course_speed
        self.antenna_offset = antenna_offset

        self.state = np.zeros(3)
        self.covariance = np.diag([gnss_noise ** 2, gnss_noise ** 2, initial_heading_noise ** 2])

    @property
    def position(self):
        """East and north of the rover centre (m)."""
        return self.state[:2]

    @property
    def heading(self):
        """Heading in radians anticlockwise from east."""
        return self.state[2]

    def lever_arm(self):
        """World-frame vector from base_link to the GNSS antenna."""
        heading = self.state[2]
        return self.antenna_offset * np.array([np.cos(heading), np.sin(heading)])

    def antenna_to_base(self, antenna_position):
        """Rover centre position for a given antenna position."""
        return antenna_position - self.lever_arm()

    def predict(self, speed, yaw_rate, time_step):
        """Moves the state forward by one time step using speed and yaw rate."""
        heading = self.state[2]
        self.state += time_step * np.array([speed * np.cos(heading), speed * np.sin(heading), yaw_rate])
        self.state[2] = wrap_to_pi(self.state[2])

        jacobian = np.eye(3)
        jacobian[0, 2] = -speed * np.sin(heading) * time_step
        jacobian[1, 2] = speed * np.cos(heading) * time_step
        self.covariance = jacobian @ self.covariance @ jacobian.T + self.process_noise * time_step

    def _correct(self, innovation, observation_matrix, noise):
        """Standard Kalman correction of state and covariance for one measurement."""
        innovation_covariance = observation_matrix @ self.covariance @ observation_matrix.T + noise
        gain = self.covariance @ observation_matrix.T @ np.linalg.inv(innovation_covariance)
        self.state = self.state + gain @ innovation
        self.state[2] = wrap_to_pi(self.state[2])
        self.covariance = (np.eye(3) - gain @ observation_matrix) @ self.covariance

    def update_position(self, antenna_position):
        """The antenna is what GNSS measures, so its lever arm enters the observation model."""
        arm_x, arm_y = self.lever_arm()
        observation_matrix = np.array([[1.0, 0.0, -arm_y], [0.0, 1.0, arm_x]])
        self._correct(antenna_position - self.state[:2] - self.lever_arm(), observation_matrix,
                      self.gnss_noise)

    def update_course(self, antenna_velocity, yaw_rate):
        """GNSS course bounds yaw drift once moving; the antenna's swing about base_link is removed first."""
        arm_x, arm_y = self.lever_arm()
        velocity = antenna_velocity - yaw_rate * np.array([-arm_y, arm_x])
        speed = np.linalg.norm(velocity)
        if speed < self.minimum_course_speed:
            return
        innovation = np.array([wrap_to_pi(np.arctan2(velocity[1], velocity[0]) - self.state[2])])
        self._correct(innovation, np.array([[0.0, 0.0, 1.0]]),
                      np.array([[self.course_noise * (self.minimum_course_speed / speed) ** 2]]))
