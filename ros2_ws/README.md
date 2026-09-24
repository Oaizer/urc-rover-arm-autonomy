# URC ROS 2 components

**Local Linux/RViz test environment:** see [LINUX_TESTING.md](LINUX_TESTING.md).
Run `start_sim.ps1` in PowerShell to build and open the hardware-free arm demo.
The complete perception-to-motion hover demo is documented in [AUTONOMY.md](AUTONOMY.md).
The controller-manager layout and hardware boundary are in [ROS2_CONTROL.md](ROS2_CONTROL.md).
Two 10-second recordings of the running mock demo are in [media/README.md](media/README.md).

Reusable foundations for the keyboard and physical-key missions. This workspace
runs a simulated tag-acquisition and hover mission. Competition typing/key
handling and real motor actuation remain separate work.

## Packages

| Package | What it does |
|---|---|
| `urc_interfaces` | IK/FK services, MoveArm action, detections, tag map and panel observations |
| `urc_kinematics` | Full-pose Y–Z–Z–X–Z–X IK and FK, metres/radians |
| `urc_can` | Simulated joint feedback or query-only moteus CAN feedback |
| `urc_perception` | Latest-frame USB capture and calibrated/detection-only ArUco processing |
| `urc_bringup` | Launch the reusable components together |
| `urc_description` | Generate the visual arm URDF from the same model as IK |
| `urc_autonomy` | Timestamped tag mapping, fixture registration, hover mission and motion supervision |
| `urc_simulation` | Interactive RViz, synthetic camera/environment and ros2_control mock arm |

The earlier standalone scripts in `outputs/` are unchanged. This is a separate
workspace; copying it does not require the older IK project's absolute path.

## Safety and scope

- Default CAN mode is stationary simulation. The real transport only queries
  controllers. It has no position-command topic and never sends stop/torque-disable
  messages, including on shutdown. A gravity-loaded arm needs a tested stop policy.
- IK returns a mathematical pose solution, not a collision-checked trajectory or
  permission to move. Geometry, limits, tool and motor calibration are not yet
  commissioned. Preserve the documented legacy arm-frame convention.
- Detection without intrinsics still returns IDs and pixel corners. Metric poses
  require matching-resolution calibration and explicitly configured marker sizes.
  Consumers must check `pose_valid`; ambiguous poses must not drive the arm.
- The simulator supplies an explicitly synthetic camera-to-J5 transform.
  Real-camera use requires measured calibration before transforming observations.
- No lock/key mission, grasping, collision planner, MoveIt plugin, physical
  ros2_control hardware plugin, force control or video web UI is included.

## Build on Ubuntu 24.04 / ROS 2 Jazzy

Run in a Linux terminal, from this `ros2_ws` directory (not PowerShell's system32):

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

ROS 2 Jazzy and the colcon/rosdep tools must already be installed. Raspberry Pi OS
is not interchangeable with this Ubuntu binary setup. Deployment on the Pi still
needs its OS/dependencies checked; a successful WSL build is not a Pi benchmark.

## Hardware-free tests

Windows or Linux with Python, NumPy, SciPy, pytest, PyYAML and OpenCV ArUco support:

```bash
python test_core.py
```

These tests do not require ROS, CAN, a camera or motors. After building ROS, use
the ROS integration test described below to exercise actual topics and services.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 ros_smoke_test.py
python3 autonomy_smoke_test.py
```

This starts temporary nodes on an isolated ROS domain, tests simulated feedback,
IK/FK requests and synthetic ArUco image messages, and shuts them down afterward.
It never opens a USB camera or CAN adapter.

## Run the components

After sourcing the installation:

```bash
# IK/FK services and stationary simulated joint feedback. No hardware opened.
ros2 launch urc_bringup components.launch.py
```

For a connected USB camera, explicitly select the dictionary printed on your tags:

```bash
# DICT_4X4_50 is an example, not an assumption about the physical-key task.
ros2 launch urc_bringup components.launch.py camera_enabled:=true \
  device:=/dev/video0 dictionary:=DICT_4X4_50
```

Without calibration this publishes IDs/corners and annotated images, not valid
3D poses. Topics are `/wrist_camera/image_raw`, `/wrist_camera/camera_info`,
`/wrist_camera/tag_detections`, `/wrist_camera/image_annotated`, `/joint_states`
and `/diagnostics`. Services are `/solve_ik` and `/compute_fk`.

Use `calibration_file:=/absolute/path/camera.yaml` for measured intrinsics. Set
`detector_config:=/absolute/path/detector.yaml` to supply explicit sizes:

```yaml
/**:
  ros__parameters:
    tag_sizes: ['7:0.01', '8:0.02']  # Examples only: ID:black-square side in metres
    publish_annotated: true
    annotated_fps: 5.0
```

For multiple cameras launch `urc_perception usb_aruco.launch.py` separately with
distinct `namespace`, `device` and camera `frame_id` settings; each camera needs
its own calibration. Do not start another simulated CAN publisher per camera.

Read-only physical CAN is a separate explicit invocation documented in
`src/urc_can/README.md`. Stop the simulated joint publisher before using it.
The combined launch intentionally cannot enable real motor movement.

## Next commissioning steps

1. Measure arm dimensions, limits and the actual tool transform.
2. Record controller IDs, encoder signs, zeros and position reporting ratios.
   Do not apply a gearbox ratio twice if moteus already reports output revolutions.
3. Calibrate camera intrinsics at the selected capture resolution and fixed focus.
4. Set the observed marker dictionary and each marker's measured black-square size.
   The key task's dictionary/IDs must not be inferred from the keyboard task.
5. Verify the J5 camera mount transform and time alignment independently.
6. Add a separately commissioned trajectory/controller layer, collision geometry
   and watchdog response before connecting IK to motor commands.

All deployed robot processing belongs on the Pi. Camera queues are kept shallow;
any laptop display jitter buffer must remain outside the control image path.
