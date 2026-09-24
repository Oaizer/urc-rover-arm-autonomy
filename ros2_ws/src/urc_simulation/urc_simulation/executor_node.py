"""Simulated FollowJointTrajectory backend. Sole publisher of simulated joints."""
import time
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from urc_autonomy.common import NAMES, get_model, stamp_seconds, tick, run
from urc_autonomy.motion import Motion
from .scene import START


class Executor(Node):
    def __init__(self):
        super().__init__('sim_joint_executor')
        self.model, _ = get_model(self)
        self.q, self.v = self.model.validate_positions(START), np.zeros(6)
        self.motion, self.busy, self.heartbeat_at = None, False, 0.
        self.feedback_enabled = True
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Empty, '/arm/heartbeat', lambda _: setattr(self, 'heartbeat_at', time.monotonic()), 1)
        self.create_timer(.02, self.publish)
        self.create_service(SetBool, '/sim/feedback_enabled', self.set_feedback)
        self.server = ActionServer(self, FollowJointTrajectory, '/arm/follow_joint_trajectory', self.execute,
                    goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                    callback_group=ReentrantCallbackGroup())

    def validate(self, request):
        trajectory = request.trajectory
        if list(trajectory.joint_names) != NAMES or len(trajectory.points) != 2:
            raise ValueError('sim executor requires two rest-to-rest points in J1..J6 order')
        if request.multi_dof_trajectory.points or request.path_tolerance or request.goal_tolerance:
            raise ValueError('multi-DOF/custom tolerance trajectories are unsupported by this simulator')
        stamp = stamp_seconds(trajectory.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        if stamp <= 0 or not 0 <= now - stamp <= 1.:
            raise ValueError('stale or future trajectory header')
        first, last = trajectory.points
        start, end = self.model.validate_positions(first.positions), self.model.validate_positions(last.positions)
        if np.max(np.abs(start - self.q)) > .01 or stamp_seconds(first.time_from_start) != 0.:
            raise ValueError('trajectory start does not match measured joints')
        for point in trajectory.points:
            if (len(point.velocities) != 6 or len(point.accelerations) != 6
                    or not np.isfinite([*point.velocities, *point.accelerations]).all()
                    or np.max(np.abs([*point.velocities, *point.accelerations])) > 1e-9
                    or point.effort):
                raise ValueError('simulator supports zero-velocity/acceleration endpoints only')
        return Motion(start, end, time.monotonic(), duration=stamp_seconds(last.time_from_start))

    def accept(self, request):
        if self.busy or time.monotonic() - self.heartbeat_at > .6:
            return GoalResponse.REJECT
        try:
            self.validate(request)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def execute(self, goal):
        result = FollowJointTrajectory.Result()
        try:
            self.motion = self.validate(goal.request)
            while True:
                if goal.is_cancel_requested:
                    result.error_code, result.error_string = result.PATH_TOLERANCE_VIOLATED, 'Cancelled; simulated joints held'
                    goal.canceled()
                    break
                if time.monotonic() - self.heartbeat_at > .6:
                    result.error_code, result.error_string = result.PATH_TOLERANCE_VIOLATED, 'Controller heartbeat lost'
                    goal.abort()
                    break
                q, v, done = self.motion.sample(time.monotonic())
                self.q, self.v = q, v
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = NAMES
                feedback.actual = JointTrajectoryPoint(positions=q.tolist(), velocities=v.tolist())
                feedback.desired = feedback.actual
                goal.publish_feedback(feedback)
                if done:
                    result.error_code, result.error_string = result.SUCCESSFUL, 'Simulated trajectory completed'
                    goal.succeed()
                    break
                await tick(self)
        except Exception as exc:
            result.error_code, result.error_string = result.INVALID_GOAL, str(exc)
            goal.abort()
        finally:
            self.motion, self.busy, self.v = None, False, np.zeros(6)
            self.publish()
        return result

    def publish(self):
        if not self.feedback_enabled:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position, msg.velocity = NAMES, self.q.tolist(), self.v.tolist()
        self.publisher.publish(msg)

    def set_feedback(self, request, response):
        self.feedback_enabled = request.data
        response.success, response.message = True, 'Simulated feedback publication updated'
        return response


def main(args=None):
    run(Executor, args)
