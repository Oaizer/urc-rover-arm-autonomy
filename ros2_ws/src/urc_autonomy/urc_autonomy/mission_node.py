"""Look around, register a calibrated panel, and visit clearance targets."""
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from urc_interfaces.msg import PanelObservation
from urc_interfaces.action import MoveArm
from urc_kinematics.core import Pose
from .common import Feedback, get_model, pose_message, stamp_seconds, run
from .mapping import load_fixture, Registration, target_pose


class Mission(Node):
    def __init__(self):
        super().__init__('hover_mission')
        self.model, config = get_model(self)
        self.base = config['base_frame']
        path = str(Path(get_package_share_directory('urc_autonomy')) / 'config' / 'demo_panel.json')
        self.fixture = load_fixture(self.declare_parameter('fixture_path', path).value)
        self.sequence = self.declare_parameter('sequence', 'ABC').value.split()
        if len(self.sequence) == 1 and self.sequence[0] not in self.fixture['targets']:
            self.sequence = list(self.sequence[0])
        self.feedback = Feedback(self.model)
        self.observation, self.observed_at = None, 0.
        self.phase, self.detail, self.active = 'IDLE', 'Ready for a hover mission', False
        self.pending_goal, self.motion_handle, self.result = None, None, None
        self.stop_reason, self.recover = '', False
        self.stop_started = 0.
        self.index, self.scan_index, self.recoveries = 0, 0, 0
        self.wait_until, self.started, self.move_started = 0., 0., 0.
        self.plan_generation, self.move_kind = 0, ''
        self.planned_target = None
        self.reset_future = None
        self.targets = self.create_publisher(PoseStamped, '/mission/target_pose', 1)
        self.visuals = self.create_publisher(MarkerArray, '/mission/markers', 1)
        self.status = self.create_publisher(String, '/mission/status', 1)
        self.active_pub = self.create_publisher(Bool, '/mission/active', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.motion = ActionClient(self, MoveArm, '/arm/move_to_pose')
        self.reset_map = self.create_client(Trigger, '/perception/reset_map')
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(PanelObservation, '/perception/panel', self.on_panel, 1)
        self.create_subscription(String, '/mission/sequence', self.on_sequence, 1)
        self.create_service(Trigger, '/mission/start', self.start)
        self.create_service(Trigger, '/mission/abort', self.abort)
        self.create_timer(.05, self.step)
        self.create_timer(.1, self.publish)
        self.active_pub.publish(Bool(data=False))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, msg):
        self.feedback.update(msg, self.now())

    def on_panel(self, msg):
        if msg.header.frame_id == self.base:
            self.observation, self.observed_at = msg, time.monotonic()

    def on_sequence(self, msg):
        if self.active:
            self.get_logger().warning('Sequence edits rejected while mission is active')
            return
        labels = msg.data.upper().split()
        if len(labels) == 1 and labels[0] not in self.fixture['targets']:
            labels = list(labels[0])
        if not labels or len(labels) > 32 or any(i not in self.fixture['targets'] for i in labels):
            self.detail = 'Rejected sequence: use configured target labels A, B, C'
            return
        self.sequence = labels
        self.detail = 'Sequence: ' + ' '.join(labels)

    def start(self, request, response):
        if self.active or not self.feedback.fresh(self.now()) or not self.motion.server_is_ready() or not self.reset_map.service_is_ready():
            response.success, response.message = False, 'Busy or waiting for feedback/controller/mapper'
            return response
        if not self.sequence or any(i not in self.fixture['targets'] for i in self.sequence):
            response.success, response.message = False, 'Invalid configured target sequence'
            return response
        self.active, self.started = True, time.monotonic()
        self.index, self.scan_index, self.recoveries = 0, 0, 0
        self.stop_reason, self.recover = '', False
        self.active_pub.publish(Bool(data=True))
        self.reset_future = self.reset_map.call_async(Trigger.Request())
        self.phase, self.detail = 'RESET_MAP', 'Starting fresh observations'
        self.wait_until = time.monotonic() + 3.
        response.success, response.message = True, 'Hover mission started'
        return response

    def abort(self, request, response):
        if self.active:
            self.stop('Operator abort')
        response.success, response.message = True, 'Abort requested; waiting for motion cancellation'
        return response

    def finish(self, phase, detail):
        self.phase, self.detail, self.active = phase, detail, False
        self.active_pub.publish(Bool(data=False))

    def stop(self, reason, recover=False):
        if self.stop_reason:
            if not recover:
                self.recover = False
            return
        self.stop_reason, self.recover = reason, recover and self.recoveries < 2
        self.stop_started = time.monotonic()
        self.phase, self.detail = 'STOPPING', reason
        if self.motion_handle is not None:
            self.motion_handle.cancel_goal_async()
        if self.pending_goal is None and self.motion_handle is None:
            self.stopped()

    def stopped(self):
        reason = self.stop_reason
        self.motion_handle, self.result, self.pending_goal = None, None, None
        if self.recover:
            self.recoveries += 1
            self.scan_index = 0
            self.reset_future = self.reset_map.call_async(Trigger.Request())
            self.phase, self.detail = 'RESET_MAP', 'Reacquiring after: ' + reason
            self.wait_until = time.monotonic() + 3.
            self.stop_reason, self.recover = '', False
        else:
            self.finish('ABORTED' if reason == 'Operator abort' else 'FAULT', reason)

    def fresh_panel(self):
        obs = self.observation
        return (obs is not None and obs.valid and time.monotonic() - self.observed_at < .5
                and 0 <= self.now() - stamp_seconds(obs.header.stamp) < 1.)

    def send(self, pose, kind, strict=False):
        self.move_kind, self.move_started = kind, time.monotonic()
        self.pending_goal = self.motion.send_goal_async(MoveArm.Goal(target=pose, source='mission',
                                              require_orientation=strict, timeout_s=20.))
        self.phase = 'SCANNING' if kind == 'scan' else 'MOVING'
        self.detail = f'View {self.scan_index}' if kind == 'scan' else f'Hover target {self.sequence[self.index]}'
        self.targets.publish(pose)

    def poll_motion(self):
        if self.pending_goal is not None:
            if not self.pending_goal.done():
                if time.monotonic() - self.move_started > 5.:
                    # Keep cancellation attached if the action response arrives late.
                    self.pending_goal.add_done_callback(lambda f: f.result().cancel_goal_async() if f.result().accepted else None)
                    self.pending_goal = None
                    self.stop('Motion action did not acknowledge the goal')
                return True
            try:
                self.motion_handle = self.pending_goal.result()
            except Exception as exc:
                self.pending_goal = None
                self.stop('Motion action error: ' + str(exc))
                return True
            self.pending_goal = None
            if not self.motion_handle.accepted:
                self.motion_handle = None
                if self.stop_reason:
                    self.stopped()
                else:
                    self.stop('Controller rejected mission target')
                return True
            self.result = self.motion_handle.get_result_async()
            if self.stop_reason:
                self.motion_handle.cancel_goal_async()
        if self.result is None:
            return False
        if not self.result.done():
            if self.stop_reason and time.monotonic() - self.stop_started > 3.:
                self.phase, self.detail = 'FAULT_STOP_UNCONFIRMED', 'Waiting for controller cancellation; arm remains reserved'
            if time.monotonic() - self.move_started > 23.:
                self.stop('Motion completion timed out')
            return True
        result = self.result.result()
        self.motion_handle, self.result = None, None
        if self.stop_reason:
            self.stopped()
        elif result.status != 4 or not result.result.success:
            self.stop('Motion failed: ' + result.result.message)
        else:
            self.phase = 'SETTLE'
            self.wait_until = time.monotonic() + .4
        return True

    def step(self):
        if not self.active:
            return
        if not self.stop_reason:
            if time.monotonic() - self.started > self.fixture['attempt_timeout_s']:
                self.stop('Attempt timeout')
            elif not self.feedback.fresh(self.now()):
                self.stop('Joint feedback stale')
            elif self.observation is not None and self.observation.reason == 'Camera stream stale or absent':
                self.stop('Camera stream lost')
            elif self.phase in ('MOVING', 'SETTLE') and self.move_kind == 'target':
                obs = self.observation
                if obs is None or time.monotonic() - self.observed_at > .7:
                    self.stop('Localization stream lost')
                elif not obs.valid or obs.generation != self.plan_generation:
                    self.stop('Panel observation invalidated during move', recover=True)
                elif self.planned_target is not None:
                    p, q = obs.pose.position, obs.pose.orientation
                    r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                    offset = self.fixture['targets'][self.sequence[self.index]] + [0, 0, self.fixture['clearance_m']]
                    updated = np.array([p.x, p.y, p.z]) + r @ offset
                    if np.linalg.norm(updated - self.planned_target) > .005:
                        self.stop('Registered target moved during approach', recover=True)
        if self.poll_motion() or not self.active or self.stop_reason:
            return
        now = time.monotonic()
        if self.phase == 'RESET_MAP':
            if self.reset_future.done():
                if not self.reset_future.result().success:
                    self.stop('Could not clear tag map')
                    return
                self.observation = None
                self.phase, self.detail = 'ACQUIRE', 'Waiting for three stable tag IDs'
                self.wait_until = now + 1.2
            elif now > self.wait_until:
                self.stop('Tag mapper reset timed out')
        elif self.phase == 'SETTLE' and now >= self.wait_until:
            if self.move_kind == 'target':
                self.index += 1
                self.scan_index = 0
                if self.index >= len(self.sequence):
                    self.finish('COMPLETE', f'Visited {len(self.sequence)} hover targets; no contact/typing performed')
                    return
            self.phase, self.detail = 'ACQUIRE', 'Refreshing panel registration'
            self.wait_until = now + 1.
        elif self.phase == 'ACQUIRE':
            if self.fresh_panel():
                self.phase, self.detail = 'PLAN', 'Registered panel; selecting target'
            elif now >= self.wait_until:
                offsets = self.fixture['scan_offsets']
                if self.scan_index >= len(offsets):
                    self.stop('Scan exhausted without a fresh registered panel')
                    return
                yaw, pitch = offsets[self.scan_index]
                q = np.array(self.fixture['home_joints'], float)
                q[0] += yaw
                q[4] += pitch
                self.scan_index += 1
                self.send(pose_message(self.model.forward(q), self.base, self.get_clock().now().to_msg()), 'scan', strict=True)
        elif self.phase == 'PLAN':
            if not self.fresh_panel():
                self.phase, self.wait_until = 'ACQUIRE', now + 1.
                return
            obs = self.observation
            p, q = obs.pose.position, obs.pose.orientation
            registration = Registration(True, '', position=np.array([p.x, p.y, p.z]),
                                        rotation=Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix())
            position, rotation = target_pose(registration, self.fixture, self.sequence[self.index])
            pose = pose_message(Pose(position, Rotation.from_matrix(rotation).as_quat()), self.base, self.get_clock().now().to_msg())
            self.plan_generation = obs.generation
            self.planned_target = position.copy()
            self.send(pose, 'target')

    def publish(self):
        self.status.publish(String(data=f'{self.phase} | {self.detail} | {self.index}/{len(self.sequence)} | recoveries={self.recoveries}'))
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = self.base, self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type = 'mission_status', 0, Marker.TEXT_VIEW_FACING
        marker.pose.orientation.w = 1.
        marker.pose.position.x, marker.pose.position.y = .35, .72
        marker.scale.z = .021
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .8, .7, 1., 1.
        marker.text = f'{self.phase}: {self.index}/{len(self.sequence)} hover targets'
        marker.lifetime = Duration(seconds=.3).to_msg()
        self.visuals.publish(MarkerArray(markers=[marker]))


def main(args=None):
    run(Mission, args)
