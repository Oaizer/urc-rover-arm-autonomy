# Verification

## 2026-09-24 ros2_control migration

- Installed ROS 2 Jazzy `ros2_control`, `joint_trajectory_controller`, and
  `joint_state_broadcaster` into the existing Ubuntu 24.04 WSL environment.
- The simulator now loads `mock_components/GenericSystem` through
  `controller_manager`. Both standard controllers activated; the joint-state
  broadcaster was the sole `/joint_states` publisher. The mock exposes six
  position/velocity command and state pairs. No CAN or physical motors ran.
- `sim_smoke_test.py` passed through the standard trajectory action: IK-driven
  motion, speed bound, TF, image detections, pause and movable panel.
- `autonomy_smoke_test.py` passed: scanning with hidden tags, registration,
  three hover targets, panel relocation recovery, mission ownership, action
  cancellation, joint-feedback loss via broadcaster deactivation, and camera loss.
  The test explicitly waits for both controllers to become active before starting.
- `test_core.py`: Windows 148 passed, 2 skipped; Ubuntu 149 passed, 1 skipped.
  The new test checks that the mock URDF interfaces and controller configuration
  agree. `ros_smoke_test.py` remains a separate IK/perception/CAN test.
- The first mock configuration inferred velocity by differentiating position;
  it reported 0.373 rad/s during a planned 0.35 rad/s motion. Explicit
  position/velocity commands and mirrored state removed that discrepancy.
- The real moteus `SystemInterface` plugin, measured limits, safety response,
  camera extrinsics and physical mission are still outstanding.

## 2026-09-23 baseline

## Completed

- Eight packages built under ROS 2 Jazzy on WSL Ubuntu 24.04 / Python 3.12.
- Windows / OpenCV 4.13: `python test_core.py` — 147 passed, 2 skipped.
- Ubuntu / distro OpenCV 4.6, ROS sourced: `python3 test_core.py` — 148 passed,
  1 skipped. Optional installed-moteus frame-encoding test skipped because that
  optional dependency was absent. Fake transport tests ran. Windows also skipped
  the ROS-specific test because native Windows Python has no rclpy.
- `python3 ros_smoke_test.py` passed with generated messages and installed nodes:
  six simulated joints, full-pose FK/IK roundtrip, invalid quaternion rejection,
  synthetic ArUco image publication/detection, original image header preserved,
  and metric pose withheld without calibration.
- Orientation-fallback regression: a near-extension target cannot be reached
  with the requested flat wrist, succeeds at the same position when orientation
  is relaxed, reports the achieved pose, and fails when `require_orientation`
  is true. Covered both the pure solver and the generated ROS service.
- Joint-step regression: a full-pose target reachable only by a distant wrist
  branch now yields a nearby free-orientation solution within the specified
  per-joint limit; strict-orientation mode rejects it.
- Combined default launch started IK and simulated CAN and both nodes shut down
  cleanly on SIGINT. Camera activation remains explicit and was not hardware-tested.
- `sim_smoke_test.py` passed: rendered image -> actual ArUco detections, one
  simulated joint publisher, no CAN node, complete arm/camera TF, full-pose IK
  driving smooth joint movement, speed bounds, pause, and interactive panel
  displacement removing tags from the camera field of view.
  The cold-start readiness timeout is 60 seconds on the Windows-mounted WSL
  workspace; steady-state motion timing checks retain their 20-second limit.
- URDF transforms numerically match the existing IK and J5/camera frames over
  zero, start and randomized joint configurations. Quintic motion speed and
  acceleration limits were checked numerically for rest-to-rest trajectories.
- Installed-graph `autonomy_smoke_test.py` passed: rendered-pixel registration
  within 2 mm in the synthetic scene, panel relocation (0.39 s reacquisition in
  this run), look-around after hidden tags, manual-goal rejection during autonomy,
  three hover targets, mid-approach panel movement/reacquisition, action cancellation,
  stale joint-feedback stop and camera-loss fault. No source-path import override
  was used for this test.
- Partial-view unit tests combine different observed IDs from two camera views;
  relocated, ambiguous, inconsistent and collinear registrations are checked.
- WSL wall-clock steps of roughly 0.94 s were observed during tests. They correctly
  triggered freshness faults before the simulation clock was added. The passing
  graph test experienced another 0.93 s host-clock step without a false stale fault.
  Simulation now uses a steady `/clock`; real hardware freshness limits were not relaxed.
- Local image transport uses a 16 MiB Fast DDS shared-memory segment. Perception
  remains full-resolution mono; only the annotated preview is half-resolution/5 Hz.
- The new graph was opened in WSLg RViz and independently rendered in Xvfb
  (`rviz-autonomy.png`). The visible graph completed A/B/C with zero recoveries
  while RViz and the annotated preview were running.
- Windows `start_sim.ps1 -NoBuild` opened Linux RViz through WSLg with OpenGL 4.5.
  A private Xvfb visual check confirmed the arm, camera image, detected tags and
  interactive controls. Fixed Jazzy's `Interactive Markers Namespace` setting
  (the old `Update Topic` key does not connect the controls).

RViz shutdown caveat: this installed RViz build sometimes hangs during SIGINT
cleanup (launch then terminates it), and the private Xvfb instance emitted a
glibc assertion while shutting down. The running visualization rendered correctly;
the ROS simulation nodes shut down cleanly. This is not a motor-control issue.

The Linux test exposed the OpenCV 4.6 DetectorParameters constructor issue; the
implementation now uses the legacy factory when available. Tests run against
both OpenCV API generations. The ROS test waits for fresh feedback rather than
mistaking an expected startup STALE diagnostic for an operational failure.

## Not yet verified

Pi performance, USB timing, physical tag accuracy, exposure-time synchronization,
CAN-FD adapter wiring/timing, installed moteus version compatibility, real motor
feedback, encoder calibration and loaded-arm behavior remain hardware checks.

No test commanded a motor, opened a physical camera, or connected to CAN.
Simulation does not certify contact safety. IK has no collision checks and real
motion is intentionally absent from this workspace.

## Local environment change

Installed `python3-colcon-common-extensions` and its dependencies into the existing
Ubuntu-24.04 WSL distribution to build the packages. No Pi changes were made.
The Windows Python environment was not modified.
Added Xvfb and its X11/font dependencies inside WSL for private GUI rendering QA.
Installed `ros-jazzy-control-msgs` for the standard trajectory action interface.
The testbed runs on localhost-only ROS domain 71, separate from the hardware.
