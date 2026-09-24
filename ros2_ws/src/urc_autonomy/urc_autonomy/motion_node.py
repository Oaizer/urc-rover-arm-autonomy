"""Pose goals -> IK -> bounded FollowJointTrajectory actions."""
import time
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty, Bool
from std_srvs.srv import Trigger, SetBool
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from urc_interfaces.action import MoveArm
from urc_interfaces.srv import SolveIK
from urc_kinematics.core import Pose
from .common import NAMES, Feedback, get_model, pose_message, stamp_seconds, tick, run
from .motion import Motion


class Controller(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.model, config = get_model(self)
        self.base = config['base_frame']
        self.feedback = Feedback(self.model)
        self.busy, self.paused, self.autonomous = False, False, False
        self.generation = 0
        self.backend_goal = None
        self.state = 'READY | awaiting target'
        self.group = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/mission/active', lambda m: setattr(self, 'autonomous', m.data), retained)
        self.status = self.create_publisher(String, '/arm/status', 1)
        self.heartbeat = self.create_publisher(Empty, '/arm/heartbeat', 1)
        self.achieved = self.create_publisher(PoseStamped, '/arm/achieved_pose', 1)
        self.create_timer(.1, self.publish)
        self.ik = self.create_client(SolveIK, '/solve_ik', callback_group=self.group)
        trajectory_action = self.declare_parameter('trajectory_action', '/arm/follow_joint_trajectory').value
        if not isinstance(trajectory_action, str) or not trajectory_action.startswith('/'):
            raise ValueError('trajectory_action must be an absolute ROS action name')
        self.backend = ActionClient(self, FollowJointTrajectory, trajectory_action, callback_group=self.group)
        self.server = ActionServer(self, MoveArm, '/arm/move_to_pose', self.execute,
                                   goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        self.client = ActionClient(self, MoveArm, '/arm/move_to_pose', callback_group=self.group)
        self.create_subscription(PoseStamped, '/arm/target_pose', self.on_target, 1)
        self.create_service(SetBool, '/arm/pause', self.pause)
        self.create_service(Trigger, '/arm/reset', self.reset)
        self.create_service(Trigger, '/arm/demo', self.demo)
        # Compatibility aliases for existing demo scripts.
        self.create_service(SetBool, '/sim/pause', self.pause)
        self.create_service(Trigger, '/sim/reset', self.reset)
        self.create_service(Trigger, '/sim/demo', self.demo)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, msg):
        self.feedback.update(msg, self.now())

    def publish(self):
        self.status.publish(String(data=self.state))
        self.heartbeat.publish(Empty())

    def accept(self, goal):
        try:
            p, q = goal.target.pose.position, goal.target.pose.orientation
            Pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            valid = (goal.target.header.frame_id == self.base
                     and 0 <= self.now() - stamp_seconds(goal.target.header.stamp) <= 2.
                     and stamp_seconds(goal.target.header.stamp) > 0
                     and np.isfinite([goal.max_joint_delta_rad, goal.timeout_s]).all()
                     and 0 <= goal.max_joint_delta_rad <= np.pi
                     and 0 <= goal.timeout_s <= 120.)
        except (ValueError, TypeError):
            valid = False
        now = self.now()
        feedback_fresh = self.feedback.fresh(now)
        if (not valid or self.busy or self.paused or not feedback_fresh
                or (self.autonomous and goal.source != 'mission')):
            if not self.busy:
                if not valid:
                    reason = 'invalid/stale target'
                elif self.paused:
                    reason = 'paused'
                elif not feedback_fresh:
                    reason = f'joint feedback stale (ROS age={now-self.feedback.stamp:.3f}s, receipt age={time.monotonic()-self.feedback.received:.3f}s)'
                else:
                    reason = 'autonomy owns the arm'
                self.state = 'REJECTED | ' + reason
                self.get_logger().warning(self.state)
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def wait(self, future, goal, deadline):
        while not future.done():
            self.check(goal, deadline)
            await tick(self)
        self.check(goal, deadline)
        return future.result()

    def check(self, goal, deadline):
        if goal.is_cancel_requested or self.paused:
            raise RuntimeError('Motion cancelled/paused')
        if time.monotonic() >= deadline:
            raise RuntimeError('Motion deadline exceeded')
        if not self.feedback.fresh(self.now()):
            raise RuntimeError('Joint feedback stale; cancelling trajectory')

    async def stop_backend(self):
        handle, self.backend_goal = self.backend_goal, None
        if handle is None or not handle.accepted:
            return
        future = handle.cancel_goal_async()
        end = time.monotonic() + 1.
        while not future.done() and time.monotonic() < end:
            await tick(self)
        future = handle.get_result_async()
        end = time.monotonic() + 1.
        while not future.done() and time.monotonic() < end:
            await tick(self)

    async def execute(self, goal):
        self.generation += 1
        generation = self.generation
        result = MoveArm.Result()
        deadline = time.monotonic() + (goal.request.timeout_s or 20.)
        try:
            self.check(goal, deadline)
            if not self.ik.service_is_ready() or not self.backend.server_is_ready():
                raise RuntimeError('IK or trajectory executor unavailable')
            seed = self.feedback.q.copy()
            limit = goal.request.max_joint_delta_rad or 1.5
            self.state = 'SOLVING | requested pose, then free wrist if permitted'
            goal.publish_feedback(MoveArm.Feedback(phase=self.state,
                                  current_pose=pose_message(self.model.forward(seed), self.base, self.get_clock().now().to_msg())))
            request = SolveIK.Request(target=goal.request.target.pose, seed=seed.tolist(),
                                      require_orientation=goal.request.require_orientation, max_joint_delta_rad=limit)
            answer = await self.wait(self.ik.call_async(request), goal, deadline)
            if not answer.success:
                raise RuntimeError('IK rejected: ' + answer.message)
            target = self.model.validate_positions(answer.positions)
            if np.max(np.abs(target - seed)) > limit + 1e-8 or np.max(np.abs(self.feedback.q - seed)) > .01:
                raise RuntimeError('Joint step or seed changed during planning')
            trajectory = Motion(seed, target, 0.)
            if time.monotonic() + trajectory.duration > deadline:
                raise RuntimeError('Insufficient time for bounded trajectory')
            request = FollowJointTrajectory.Goal()
            request.trajectory.header.stamp = self.get_clock().now().to_msg()
            request.trajectory.joint_names = NAMES
            for q, seconds in [(seed, 0.), (target, trajectory.duration)]:
                request.trajectory.points.append(JointTrajectoryPoint(positions=q.tolist(), velocities=[0.]*6,
                        accelerations=[0.]*6, time_from_start=Duration(seconds=seconds).to_msg()))
            pending = self.backend.send_goal_async(request)

            def late_reply(future):
                try:
                    handle = future.result()
                    if generation != self.generation:
                        if handle.accepted:
                            handle.cancel_goal_async()
                    else:
                        self.backend_goal = handle
                except Exception:
                    pass
            pending.add_done_callback(late_reply)
            self.backend_goal = await self.wait(pending, goal, deadline)
            if not self.backend_goal.accepted:
                raise RuntimeError('Trajectory executor rejected goal')
            mode = 'wrist orientation relaxed' if answer.orientation_relaxed else 'requested orientation kept'
            self.state = 'MOVING | ' + mode
            pending = self.backend_goal.get_result_async()
            while not pending.done():
                self.check(goal, deadline)
                goal.publish_feedback(MoveArm.Feedback(phase=self.state,
                    current_pose=pose_message(self.model.forward(self.feedback.q), self.base, self.get_clock().now().to_msg())))
                await tick(self, .05)
            self.check(goal, deadline)
            response = pending.result()
            if response.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL or response.status != 4:
                raise RuntimeError('Trajectory failed: ' + response.result.error_string)
            # Allow the final measurement to arrive after the action result.
            finish = min(deadline, time.monotonic() + .5)
            while np.max(np.abs(self.feedback.q - target)) > .002 and time.monotonic() < finish:
                self.check(goal, deadline)
                await tick(self)
            if np.max(np.abs(self.feedback.q - target)) > .002:
                raise RuntimeError('Measured joints did not reach planned target')
            result.success, result.message = True, 'target reached | ' + mode
            result.orientation_relaxed = answer.orientation_relaxed
            result.achieved_pose = pose_message(self.model.forward(self.feedback.q), self.base, self.get_clock().now().to_msg())
            self.achieved.publish(result.achieved_pose)
            self.backend_goal = None
            self.state = 'READY | ' + result.message
            goal.succeed()
        except Exception as exc:
            self.generation += 1
            await self.stop_backend()
            result.success, result.message = False, str(exc)
            self.state = 'STOPPED | ' + result.message
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
        finally:
            self.generation += 1
            self.busy = False
        return result

    def submit(self, pose):
        if self.busy or self.paused or self.autonomous or not self.client.server_is_ready():
            return False
        future = self.client.send_goal_async(MoveArm.Goal(target=pose, source='manual'))
        future.add_done_callback(lambda f: f.result().get_result_async() if f.result().accepted else None)
        return True

    def on_target(self, msg):
        if not self.submit(msg) and not self.busy:
            self.state = 'REJECTED | manual command unavailable during autonomy/pause'

    def pause(self, request, response):
        self.paused = request.data
        if not self.busy:
            self.state = 'PAUSED' if self.paused else 'READY'
        response.success, response.message = True, 'Pause set; active action will cancel' if self.paused else 'Ready for a new command'
        return response

    def reset(self, request, response):
        q = [0., .5, -1., 0., .5, 0.]
        response.success = self.submit(pose_message(self.model.forward(q), self.base, self.get_clock().now().to_msg()))
        response.message = 'Home target submitted' if response.success else 'Busy, paused or autonomous'
        return response

    def demo(self, request, response):
        q = [.12, .42, -.88, .12, .46, .08]
        if self.feedback.q is not None and np.linalg.norm(self.feedback.q - q) < .03:
            q = [0., .5, -1., 0., .5, 0.]
        response.success = self.submit(pose_message(self.model.forward(q), self.base, self.get_clock().now().to_msg()))
        response.message = 'Demo target submitted' if response.success else 'Busy, paused or autonomous'
        return response


def main(args=None):
    run(Controller, args)
