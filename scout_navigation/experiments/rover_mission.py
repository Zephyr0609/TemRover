"""Runs the first survey line and both turns on the real rover, bounded by step count and wall clock."""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor

from scout_navigation.scout_can_bridge import ScoutCanBridge, MODE_NAMES
from scout_navigation.survey_navigator import SurveyNavigator

STOP_STEP = 4
MISSION_TIMEOUT = 150.0


def describe(navigator):
    step = navigator.mission[navigator.step_index] if navigator.step_index < len(navigator.mission) else None
    return f'{navigator.state:16s} step {navigator.step_index}  {step[0] if step else "done"}'


def main():
    rclpy.init()
    bridge = ScoutCanBridge()
    navigator = SurveyNavigator()

    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(navigator)

    deadline = time.monotonic() + MISSION_TIMEOUT
    reported = None
    while navigator.step_index < STOP_STEP and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.001)
        if navigator.state != reported:
            reported = navigator.state
            print(f'{MISSION_TIMEOUT - (deadline - time.monotonic()):6.1f}s  {describe(navigator)}')

    navigator.command_publisher.publish(Twist())
    for _ in range(100):
        executor.spin_once(timeout_sec=0.01)

    east, north = navigator.estimator.position
    print(f'stopped at step {navigator.step_index} of {len(navigator.mission)}')
    print(f'  dead-reckoned pose  east {east:+.2f} m  north {north:+.2f} m  '
          f'heading {navigator.estimator.heading:+.3f} rad')
    print(f'  chassis mode {MODE_NAMES.get(bridge.control_mode, bridge.control_mode)}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
