"""End-to-end rendered perception -> mapping -> mission -> IK -> action -> joints.

Uses an isolated ROS domain, never launches CAN, and terminates its own graph.
"""
import os
from pathlib import Path
os.environ['ROS_DOMAIN_ID'] = '180'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).resolve().parent / 'src/urc_simulation/config/fastdds.xml')
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String, Int32MultiArray
from std_srvs.srv import Trigger, SetBool
from controller_manager_msgs.srv import ListControllers, SwitchController
from visualization_msgs.msg import InteractiveMarkerFeedback
from urc_interfaces.msg import PanelObservation, TagDetections
from urc_interfaces.action import MoveArm
from urc_autonomy.common import get_model, pose_message

LAUNCH_PROCESS = None


def spin(node, condition, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if LAUNCH_PROCESS is not None and LAUNCH_PROCESS.poll() is not None:
            raise AssertionError('Simulation launch exited before the test completed')
        rclpy.spin_once(node, timeout_sec=.02)
        if condition():
            return
    raise AssertionError('Timed out waiting for ' + getattr(condition, '__name__', 'ROS condition'))


def call(node, service_type, name, request):
    client = node.create_client(service_type, name)
    assert client.wait_for_service(timeout_sec=5), name
    future = client.call_async(request)
    spin(node, future.done)
    value = future.result()
    node.destroy_client(client)
    assert value.success, value.message
    return value


def switch_broadcaster(node, activate):
    client = node.create_client(SwitchController, '/controller_manager/switch_controller')
    assert client.wait_for_service(timeout_sec=5)
    name = 'joint_state_broadcaster'
    request = SwitchController.Request(
        activate_controllers=[name] if activate else [],
        deactivate_controllers=[] if activate else [name],
        strictness=SwitchController.Request.STRICT,
    )
    request.timeout.sec = 3
    future = client.call_async(request)
    spin(node, future.done)
    assert future.result().ok, future.result().message
    node.destroy_client(client)


def main():
    global LAUNCH_PROCESS
    rclpy.init()
    node = rclpy.create_node('autonomy_graph_test', parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)])
    process = subprocess.Popen(['ros2', 'launch', 'urc_simulation', 'demo.launch.py',
                                'rviz:=false', 'auto_demo:=false'], start_new_session=True)
    LAUNCH_PROCESS = process
    states, panels, status, arm = [], [], [], []
    streams = {}
    try:
        def stream(name, msg):
            previous = streams.get(name)
            streams[name] = (time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                             len(msg.detections) if hasattr(msg, 'detections') else 0,
                             max(previous[3], time.monotonic()-previous[0]) if previous else 0.)
        node.create_subscription(Image, '/sim_camera/image_raw', lambda m: stream('image', m), qos_profile_sensor_data)
        node.create_subscription(TagDetections, '/sim_camera/tag_detections', lambda m: stream('detections', m), qos_profile_sensor_data)
        def joint_state(msg):
            states.append(msg)
            stream('joints', msg)
        node.create_subscription(JointState, '/joint_states', joint_state, 10)
        clock_offset = [time.time() - time.monotonic()]
        def check_clock():
            offset = time.time() - time.monotonic()
            if abs(offset - clock_offset[0]) > .1:
                print(f'CLOCK STEP {offset-clock_offset[0]:.3f}s', flush=True)
            clock_offset[0] = offset
        node.create_timer(.05, check_clock)
        node.create_subscription(PanelObservation, '/perception/panel', panels.append, 1)
        node.create_subscription(String, '/mission/status', lambda m: status.append(m.data), 10)
        node.create_subscription(String, '/arm/status', lambda m: arm.append(m.data), 10)
        spin(node, lambda: states and panels and panels[-1].valid and status, timeout=90)
        assert node.count_publishers('/joint_states') == 1
        assert 'urc_can' not in node.get_node_names()
        controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        assert controllers.wait_for_service(timeout_sec=5)
        required = {'joint_state_broadcaster': 'active', 'arm_trajectory_controller': 'active'}
        controller_deadline = time.monotonic() + 40
        while True:
            listed = controllers.call_async(ListControllers.Request())
            spin(node, listed.done)
            found = {c.name: c.state for c in listed.result().controller}
            if all(found.get(name) == state for name, state in required.items()):
                break
            assert time.monotonic() < controller_deadline, found
            spin(node, lambda: False, timeout=.2)
        node.destroy_client(controllers)
        observed = panels[-1].pose.position
        np.testing.assert_allclose([observed.x, observed.y, observed.z], [.82, .19, 0], atol=.002)
        print('PASS rendered image -> calibrated registration in arm base (<=2 mm)', flush=True)

        publisher = node.create_publisher(InteractiveMarkerFeedback, '/sim/controls/feedback', 10)
        spin(node, lambda: publisher.get_subscription_count() > 0)
        # InteractiveMarkerServer reserves a new marker's client for one second.
        ready_at = time.monotonic()
        spin(node, lambda: time.monotonic() - ready_at > 1.1)
        old_generation = panels[-1].generation
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = 'arm_base_legacy'
        feedback.marker_name, feedback.client_id = 'tag_panel', 'autonomy_test'
        feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z = .80, .195, .008
        feedback.pose.orientation.y, feedback.pose.orientation.w = -2**-.5, 2**-.5
        changed_at = time.monotonic()
        publisher.publish(feedback)
        spin(node, lambda: panels[-1].generation > old_generation and panels[-1].valid)
        observed = panels[-1].pose.position
        np.testing.assert_allclose([observed.x, observed.y, observed.z], [.80, .195, .008], atol=.002)
        print(f'PASS panel relocation invalidates old map; reacquired in {time.monotonic()-changed_at:.2f}s', flush=True)

        action = ActionClient(node, MoveArm, '/arm/move_to_pose')
        assert action.wait_for_server(timeout_sec=5)
        model, config = get_model(node)
        visibility = node.create_publisher(Int32MultiArray, '/sim/visible_tag_ids', 1)
        spin(node, lambda: visibility.get_subscription_count() > 0)
        visibility.publish(Int32MultiArray(data=[]))
        call(node, Trigger, '/mission/start', Trigger.Request())
        spin(node, lambda: status[-1].startswith('SCANNING') and 'View 2' in status[-1], timeout=30)
        visibility.publish(Int32MultiArray(data=[0, 1, 2, 3]))
        spin(node, lambda: status and status[-1].startswith('MOVING'), timeout=30)
        print('PASS look-around scan acquires the panel after visibility is restored', flush=True)
        manual = MoveArm.Goal(target=pose_message(model.forward(states[-1].position), config['base_frame'], node.get_clock().now().to_msg()), source='manual')
        pending = action.send_goal_async(manual)
        spin(node, pending.done)
        assert not pending.result().accepted
        print('PASS manual action rejected during autonomous ownership', flush=True)
        feedback.pose.position.x = .79
        publisher.publish(feedback)
        spin(node, lambda: status[-1].startswith(('COMPLETE', 'FAULT', 'ABORTED')), timeout=65)
        assert status[-1].startswith('COMPLETE'), status[-1]
        assert '3/3' in status[-1]
        assert 'recoveries=1' in status[-1], status[-1]
        assert any(s.startswith('MOVING') for s in arm)
        print('PASS three observation-derived hover targets completed through ROS IK/motion actions', flush=True)
        print('PASS moving the panel during approach triggers reacquisition before counting the target', flush=True)

        def start_manual(q):
            pose = pose_message(model.forward(q), config['base_frame'], node.get_clock().now().to_msg())
            pending = action.send_goal_async(MoveArm.Goal(target=pose, source='manual', timeout_s=20.))
            spin(node, pending.done)
            handle = pending.result()
            assert handle.accepted
            return handle

        # Let the ownership release and final joint feedback arrive.
        count = len(states)
        spin(node, lambda: len(states) > count + 5)
        start_q = np.asarray(states[-1].position)
        target_q = start_q.copy()
        target_q[0] += .3
        handle = start_manual(target_q)
        result = handle.get_result_async()
        spin(node, lambda: np.linalg.norm(np.asarray(states[-1].position) - start_q) > .015)
        cancel = handle.cancel_goal_async()
        spin(node, cancel.done)
        spin(node, result.done)
        assert result.result().status == 5 and not result.result().result.success
        count = len(states)
        spin(node, lambda: len(states) > count + 5)
        frozen = np.asarray(states[-1].position)
        count = len(states)
        spin(node, lambda: len(states) > count + 15)
        np.testing.assert_allclose(states[-1].position, frozen, atol=1e-9)
        print('PASS action cancellation stops joints with no delayed movement', flush=True)

        target_q = np.asarray(states[-1].position).copy()
        target_q[0] -= .3
        handle = start_manual(target_q)
        result = handle.get_result_async()
        count = len(states)
        spin(node, lambda: len(states) > count + 5)
        switch_broadcaster(node, False)
        spin(node, result.done)
        assert not result.result().result.success and 'stale' in result.result().result.message.lower()
        switch_broadcaster(node, True)
        count = len(states)
        spin(node, lambda: len(states) > count + 10)
        print('PASS stale joint feedback aborts active motion', flush=True)

        call(node, SetBool, '/sim/camera_enabled', SetBool.Request(data=False))
        spin(node, lambda: panels and not panels[-1].valid and 'Camera stream' in panels[-1].reason)
        call(node, Trigger, '/mission/start', Trigger.Request())
        spin(node, lambda: status[-1].startswith('FAULT') and 'Camera' in status[-1])
        print('PASS camera loss invalidates localization and faults the mission', flush=True)
        call(node, SetBool, '/sim/camera_enabled', SetBool.Request(data=True))
        assert process.poll() is None
        print('Autonomy graph smoke test passed', flush=True)
    except Exception:
        print('Last mission states:', status[-5:], flush=True)
        print('Last arm states:', arm[-5:], flush=True)
        print('Camera stream ages/count/max gap:', {k: (time.monotonic()-v[0], node.get_clock().now().nanoseconds*1e-9-v[1], v[2], v[3]) for k,v in streams.items()}, flush=True)
        if panels:
            print('Last panel:', panels[-1].valid, panels[-1].reason, list(panels[-1].tag_ids), flush=True)
        raise
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
