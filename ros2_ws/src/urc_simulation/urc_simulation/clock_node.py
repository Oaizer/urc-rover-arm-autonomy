"""Real-time-paced simulation clock, independent of host wall-clock corrections."""
import time
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from urc_autonomy.common import run


class SimulationClock(Node):
    def __init__(self):
        super().__init__('simulation_clock')
        self.started = time.monotonic_ns()
        self.publisher = self.create_publisher(ClockMessage, '/clock', 1)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.01, self.publish, clock=self.steady)

    def publish(self):
        # Start above zero: ROS treats zero as an uninitialized simulation clock.
        elapsed = time.monotonic_ns() - self.started + 1_000_000_000
        self.publisher.publish(ClockMessage(clock=Time(nanoseconds=elapsed).to_msg()))


def main(args=None):
    run(SimulationClock, args)
