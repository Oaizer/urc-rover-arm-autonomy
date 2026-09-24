# ROS autonomy simulation

The camera renderer, perception, motion planning and joint executor are separate
ROS nodes. No CAN node runs in this launch.

```text
urc_simulator -- /sim_camera/image_raw --> aruco_detector
                                              |
                                 /sim_camera/tag_detections
                                              v
/joint_states --> robot_state_publisher --> TF --> tag_mapper
                                              |
                         /perception/tag_map + /perception/panel
                                              v
                                        hover_mission
                                              |
                              /arm/move_to_pose (MoveArm action)
                                              v
                                        arm_controller
                                              |
                       /solve_ik service <--> urc_kinematics
                                              |
                   /arm/follow_joint_trajectory (standard ROS action)
                                              v
                                      sim_joint_executor
                                              |
                                         /joint_states
                                              v
                                 RViz + camera renderer
```

## Run it

In Ubuntu:

```bash
cd /mnt/c/Users/oaize/Documents/Codex/2026-09-20/i-have-a-new-project-i/ros2_ws
bash scripts/run_demo.sh
```

After building once, `URC_SKIP_BUILD=1 bash scripts/run_demo.sh` starts faster.
Closing RViz stops this launch. Windows can use `start_sim.ps1 -NoBuild`.

In RViz select **Interact**, right-click either control's central cube, and
choose **Start hover mission (A B C)**. The arm acquires the panel, moves to
clearance targets A, B and C, and reports COMPLETE. This means hover targets
were reached; it does not mean characters were typed.

- Drag the panel's translation arrows or rotation rings to change the camera view.
- Green points are accumulated measured tags; the yellow rectangle is registration.
- Purple points on the fixture show A/B/C. The orange IK control follows the planned target.
- The mission status label shows acquisition, scan, move and completion.
- **Abort mission** cancels the outstanding action. **Pause / cancel arm** also
  holds the arm until **Resume arm** is selected.
- Manual target drags and sequence edits are rejected during a mission.
- **Reset arm to observation pose** is available after the mission finishes or aborts.

The demo uses a synthetic 150 x 100 mm panel and 20 mm tags with IDs 0–3 from
DICT_4X4_50. It is not the dimensions or tag layout of a competition keyboard.
The known tag centres, A/B/C offsets, hover clearance, and scan poses are in
`src/urc_autonomy/config/demo_panel.json`. Replace them with surveyed geometry
before using a real fixture. Three unlabelled points alone only establish a plane;
known ID correspondences establish the panel axes and target offsets here.

## Command it from another Ubuntu terminal

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/oaize/Documents/Codex/2026-09-20/i-have-a-new-project-i/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 service call /mission/abort std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status
```

Change the sequence while idle, then start a new mission:

```bash
ros2 topic pub --once /mission/sequence std_msgs/msg/String '{data: "C A B"}'
```

Manual pose goals use `/arm/target_pose` (`geometry_msgs/PoseStamped`) or the
`/arm/move_to_pose` action (`urc_interfaces/action/MoveArm`). A target must have
a current nonzero ROS timestamp, frame `arm_base_legacy`, metres and a unit
quaternion. `source: mission` is reserved for the mission node. Set
`require_orientation: true` when tool alignment must not be relaxed.
In this simulator, command-producing nodes must set `use_sim_time:=true` and
timestamp goals from `/clock`, not the laptop's wall clock.

The simulation publishes a steady, real-time-paced `/clock`. Every launched node,
including RViz, uses it; Windows/WSL wall-clock corrections therefore do not age
otherwise fresh observations. Real hardware launches do not use this clock.
Monotonic command deadlines and the executor heartbeat still protect against stalls.

Raw perception stays at 640 x 480 grayscale. Only the annotated preview is reduced
to 320 x 240 at 5 Hz. OpenCV uses one worker thread. The local Fast DDS profile
allocates 16 MiB per shared-memory segment and restricts UDP discovery to loopback.
It is for this laptop simulation, not a multi-computer hardware deployment.

The controller reads fresh `/joint_states`, calls IK, applies the 1.5-radian
per-joint step bound, constructs a rest-to-rest quintic trajectory, and sends
the trajectory action. It reports progress and the achieved pose. It cancels
on stale joint feedback, a deadline, pause, or action cancellation. The
simulator enforces velocity/acceleration limits and holds on controller
heartbeat loss. Cancel/hold is an instantaneous simulation operation; it is
not a physical braking model.

## Interfaces

| Name | Type | Owner |
|---|---|---|
| `/sim_camera/image_raw`, `/sim_camera/camera_info` | Image, CameraInfo topics | Synthetic environment |
| `/sim_camera/tag_detections` | TagDetections topic | Detector |
| `/perception/tag_map` | TagMap topic | Mapper |
| `/perception/panel` | PanelObservation topic | Mapper |
| `/mission/sequence`, `/mission/status` | String topics | Operator / mission |
| `/mission/target_pose` | PoseStamped topic, planned target for display | Mission |
| `/mission/start`, `/mission/abort` | Trigger services | Mission |
| `/arm/target_pose` | PoseStamped topic, manual target input | Controller |
| `/arm/move_to_pose` | MoveArm action | Controller |
| `/solve_ik`, `/compute_fk` | SolveIK, ComputeFK services | Kinematics |
| `/arm/follow_joint_trajectory` | control_msgs/FollowJointTrajectory action | Simulated executor |
| `/arm/status`, `/arm/achieved_pose` | String, PoseStamped topics | Controller |
| `/arm/pause` | SetBool service | Controller |
| `/arm/reset`, `/arm/demo` | Trigger services | Controller |
| `/joint_states` | JointState topic | Simulated executor, sole publisher |
| `/tf`, `/tf_static` | TF topics | robot_state_publisher |
| `/clock` | rosgraph_msgs/Clock topic | Steady simulation clock |

The simulation backend supports two rest-to-rest trajectory points with zero
endpoint velocities/accelerations; it rejects unsupported trajectories rather
than silently changing their meaning. The controller uses that subset of the
standard FollowJointTrajectory action. A future hardware backend should expose
the same action and joint feedback, with its own commissioned limits/watchdog.

## Registration and recovery

The mapper uses TF at the image timestamp and accepts settled observations.
Single tags need an unambiguous metric pose. When at least three configured tags
are visible together, all their image corners can resolve the board pose, even
when individual square poses are ambiguous. Ambiguous board solutions are rejected.

At least three IDs need repeated observations. Geometry error, face normals,
sample spread and age are checked before registration is marked valid. Moving
a previously mapped tag beyond 8 mm clears the whole old map. An invalidated
registration cancels a target move and allows at most two reacquisitions.
Camera or joint-feedback loss faults the mission. The attempt timeout is 90 s.

Registration is refreshed before each target. During movement the mapper retains
the last settled map; it cannot detect scene movement from observations it has
discarded because the arm is moving. This is suitable for hover commissioning,
not an assurance of contact safety.

## Failure and visibility tests

```bash
# Hide all but one tag; restore with [0, 1, 2, 3]. Empty list hides all tags.
ros2 topic pub --once /sim/visible_tag_ids std_msgs/msg/Int32MultiArray '{data: [0]}'

# Simulate a lost camera, then restore it with data: true.
ros2 service call /sim/camera_enabled std_srvs/srv/SetBool '{data: false}'

# Simulate missing joint feedback, then restore it with data: true.
ros2 service call /sim/feedback_enabled std_srvs/srv/SetBool '{data: false}'
```

After sourcing ROS and the workspace, run:

```bash
python3 test_core.py
python3 autonomy_smoke_test.py
```

The integration test uses its own ROS domain. It checks image-derived registration,
panel relocation, autonomous ownership, three hover targets, cancellation, stale
joint feedback and camera loss. The older `sim_smoke_test.py` still checks manual
IK, TF, pause and rendered-image detections.

Not implemented by this build: physical motor commands, collision/path clearance,
contact dynamics, key pressing, OCR readback, or the competition's full servicing
mission. The real tag arrangement, camera calibration and robot/tool measurements
remain commissioning inputs.
