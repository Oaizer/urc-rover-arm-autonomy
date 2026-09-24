# Local Linux + RViz test setup

For the current ROS perception-to-motion pipeline and mission controls, see
[AUTONOMY.md](AUTONOMY.md).

## What runs where

Use the existing **Ubuntu-24.04 WSL 2 distribution** on this laptop. ROS 2 Jazzy,
Python, IK, simulated joint feedback, image generation and ArUco detection run
inside Linux. RViz is a Linux GUI application displayed on Windows by WSLg.
No dual boot, separate VM, Windows ROS installation or Pi is required.

The local launcher uses ROS domain 71 and localhost-only discovery. It does not
launch the CAN node or open a physical camera. Do not change the test launch to
connect a real arm: it has no collision/contact safety controller.

## Start from Windows

Open PowerShell in this `ros2_ws` directory:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\start_sim.ps1
```

This builds the workspace and opens RViz with all test nodes. On this machine
the dependencies and initial build are already present. To skip rebuilding:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\start_sim.ps1 -NoBuild
```

The script resolves its own directory, so an absolute script path also works
from `C:\Windows\System32`. It explicitly chooses **Ubuntu-24.04**, not the
machine's default `Ubuntu` distribution. The latter may be a different installation.

Available options:

| Option | Effect |
|---|---|
| `-NoBuild` | Use the installed workspace; rebuild after editing source |
| `-NoAutoDemo` | Compatibility option; stationary startup is now the default |
| `-Headless` | Run nodes without RViz |
| `-SoftwareRendering` | Use CPU OpenGL if WSL GPU rendering fails |

Close RViz or press Ctrl+C in the launcher to stop the entire demo. A local lock
prevents two copies launched through this script from publishing joint states.
For a headless launch, stop it with Ctrl+C.

If dependencies are missing, run `setup_wsl.ps1` once. It installs packages only
inside Ubuntu-24.04 and assumes that distro already has the Jazzy apt repository
and base ROS installation. It does not install WSL itself, change the default
distro, reboot Windows, or overwrite other Linux distributions.

## Use RViz

1. The arm starts at its observation pose. Every joint is drawn from the
   simulated executor's `/joint_states`.
2. Select **Interact** in the toolbar (`i`). Drag the orange **IK target** arrows
   to translate it or the rotation rings to orient it. Release the mouse to solve
   full-pose IK and execute a bounded, smooth simulated joint move.
3. Right-click a control's central cube for **Start hover mission (A B C)**,
   **Abort mission**, **Run manual IK demo**, pause/resume, or reset.
4. Drag the **Synthetic panel** controls to translate/rotate the four tags. The
   camera image changes through perspective projection. The actual detector must
   find the tags again. Moving the panel does not automatically command the arm.
5. Watch **Wrist camera - ArUco overlay**. The panel's orange tag squares in 3D are
   labelled ground truth. Green points are accumulated metric tag observations;
   the yellow rectangle is the registered panel. The mapper can use multi-tag
   corner geometry to resolve individually ambiguous square poses.
6. Enable **Joint frames** in Displays to inspect all transforms. Drag with the
   Move Camera tool to orbit the scene; scroll to zoom.

The status label reports solving, moving, rejected or paused. Targets outside
reach and invalid poses are rejected. The IK solver searches for a result within
1.5 radians per joint of the current pose, relaxing wrist orientation when a
nearby full-pose solution is unavailable.
While moving/solving, new arm targets are rejected; wait, or pause then resume and
submit a new target. Pause is an instantaneous simulation freeze, not a model
of how a loaded physical arm brakes. Reset is a smooth move to the start pose.

The requested wrist orientation is a preference by default. IK first tries that
full pose, then retries the same TCP position with free orientation if needed.
The target marker updates to the achieved orientation, and `/sim/status` reports
whether orientation was relaxed. Joint limits, position tolerance and the demo's
joint-step limit remain in force. Strict callers of `/solve_ik` must set
`require_orientation: true` (for example when aligning a key with a lock).

## Linux commands

From Windows, enter the right distro:

```powershell
wsl -d Ubuntu-24.04
```

Then in Linux:

```bash
cd /mnt/c/Users/oaize/Documents/Codex/2026-09-20/i-have-a-new-project-i/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Use this instead of the Windows launcher, not alongside it:
bash scripts/run_demo.sh
```

In a second Linux terminal, source the same files and set the same domain, then:

```bash
ros2 topic echo /sim/status
ros2 topic echo /mission/status
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 service call /mission/abort std_srvs/srv/Trigger '{}'
ros2 service call /sim/demo std_srvs/srv/Trigger '{}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: false}'
ros2 service call /sim/reset std_srvs/srv/Trigger '{}'
```

Programs can send `geometry_msgs/PoseStamped` to `/arm/target_pose` (the old
`/sim/target_pose` relays to it). Its frame must be `arm_base_legacy`, timestamp
current and nonzero, positions metres, and orientation a unit quaternion.
The controller owns this input; the CAN node remains read-only.

## Model and data paths

```text
RViz target / hover mission -> arm_controller -> /solve_ik
                                           |
                         /arm/follow_joint_trajectory action
                                           |
                                simulated joint executor
                                           |
                                     /joint_states
                                           |
                              robot_state_publisher -> TF -> RViz arm

simulated J5 camera + movable panel -> rendered /sim_camera/image_raw
                                           |
                                  urc_perception detector
                                           |
                                tag_detections + annotated image
                                           |
                         timestamped TF -> tag mapper -> registered panel
                                           |
                                      hover mission
```

`urc_description` generates URDF directly from the same JSON model as the IK
solver. It includes the actual Y–Z–Z–X–Z–X chain and L-shaped bracket offsets.
Link widths/motor boxes are visual approximations. The demo geometry, joint
limits and tool transform remain uncommissioned.

The legacy arm frame is Y-up. A fixed rotation maps it into RViz's Z-up `world`:
`[x, y, z]_world = [x, -z, y]_legacy`. J5's output frame is at the J5 center after
its rotation, not the TCP. Robot state publisher is the only TF authority; the
kinematics node's optional TF output is disabled.

The camera transform, intrinsics, 20 mm tags and IDs 0–3 are **synthetic fixtures**,
not measured hardware values or claims about the 2027 lock task. No synthetic
tag poses are sent into the detector. Input image timestamps and joint-state
timestamps match; real USB exposure/encoder timing still needs validation.

## Keep RViz when using real feedback later

`ros2 launch urc_simulation view_arm.launch.py` starts only the model publisher
and a clean arm-only RViz view. It consumes an existing `/joint_states` source;
it does not publish fake feedback, connect CAN, or command motors. Supply your
measured model with `ik_config:=/absolute/path/measured.json` and optionally remap
`joint_states_topic`.

Stop the simulator first. Do not run two robot-state publishers for the same
frames, or mix fake and physical joint publishers. If feedback stops, the last
pose may remain on screen: RViz alone is not a safety/freshness monitor. Check
the CAN `/diagnostics` stream. Physical-device USB passthrough and networking are
separate commissioning work; this setup does not claim to test the actual bus.

## Tests

```bash
python3 test_core.py
python3 ros_smoke_test.py
python3 sim_smoke_test.py
python3 autonomy_smoke_test.py
```

The graph tests use isolated ROS domains and clean up their own processes. The
simulation test checks one joint publisher, no CAN node, rendered-image tag
detection, the arm/camera TF tree, IK-driven motion, speed bounds, pause, and
moving the panel out of view through the interactive-marker interface.

This is **kinematic and image-pipeline simulation**, not Gazebo or a physics
engine. It does not model contact, key insertion, gripper slip, arm-camera
occlusion, realistic lighting, motion blur, motor torque or controller latency.
It cannot prove collision safety, hardware precision, or Pi performance.

## Troubleshooting

- **No Linux window:** verify `wsl -d Ubuntu-24.04 -- printenv DISPLAY` returns a
  display. WSLg is required; no full Linux desktop installation is needed.
- **Black/flickering RViz:** close the demo and relaunch with `-SoftwareRendering`.
- **RViz model missing:** confirm the fixed frame is `world`, RobotModel is enabled,
  and `/joint_states` has exactly one publisher. Use the supplied RViz config.
- **No green tag poses:** check the camera overlay; tags may be out of view or
  planar pose hypotheses may be ambiguous. That rejection is intentional.
- **Changes not appearing:** omit `-NoBuild` to rebuild. Build artifacts currently
  live beside the source on the Windows mount; that is convenient but slower
  than a native Linux filesystem for larger projects.
- **RViz takes a few seconds to close:** this installed RViz build can hang in
  SIGINT cleanup; ROS launch terminates it after its shutdown timeout. The other
  simulation nodes exit normally. If closing RViz prompts to save configuration,
  discard changes unless you intentionally want a custom view.

References: [Microsoft WSL GUI support](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps),
[robot_state_publisher](https://github.com/ros/robot_state_publisher).
