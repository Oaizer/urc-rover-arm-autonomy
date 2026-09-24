# urc_kinematics

## Wrist-orientation fallback

IK tries the requested orientation first (a flat wrist when that is the supplied
target). If no full-pose solution is found within the requested joint-step limit,
it retries the same TCP position with free orientation. A reachable full pose on
a distant joint branch does not prevent this fallback. It does not change joint limits, tool geometry or position
tolerance. This is a finite search, not proof that the first pose is impossible.

`SolveIK` returns `orientation_relaxed` and `achieved_pose`; its orientation error
still measures deviation from the original requested orientation. Set request
`require_orientation=true` for strict tasks such as lock insertion. Python callers
can use `solve(..., allow_orientation_fallback=False)` for the same behavior.
Set `max_joint_delta_rad` on `SolveIK` to limit each joint's change from the seed
(zero disables the limit); Python callers can pass the same named argument.
The interactive simulator uses 1.5 rad. This is a joint-step guard, not a
collision or path-safety guarantee.

Reusable ROS 2 `ament_python` package for full-pose IK and FK of the local
Y-Z-Z-X-Z-X arm. Public inputs/outputs use **meters, radians, and XYZW unit
quaternions**. This package never commands motion. Services work without any
joint-state publisher; measured-state TF is optional and disabled by default.

## Model and commissioning

The mathematics is adapted from `arm_ik.py` in the original
`2026-09-19/i-need-to-design-an-inverse` project. `config/demo.json` converts
that project's `arm_config.json` dimensions from mm to m, and its +/-180 degree
limits to radians. Each joint rotates about its **local** Y/Z/Z/X/Z/X axis,
then translates along the rotated local offset:

`[0,d1,0], [L2,0,0], [L3x,L3y,0], [L4,0,0], [L5,0,0], [L6,0,0]`.

All joint frames align at zero; positive angles obey the right-hand rule.
The L-bracket's corner is not another joint. The explicit `tool_transform`
is the fixed transform from the endpoint AFTER L6 to TCP; demo identity
preserves the original TCP. Do not count L6 twice when adding a real tool.
At zero with the identity tool, TCP is `[0.83, 0.15, 0]` m.

**UNCOMMISSIONED:** dimensions, limits, zero offsets, axis signs and tool are
demonstration data, not surveyed hardware calibration. `commissioned: false`
causes a startup warning; changing this flag does not validate anything.
Configuration has no implicit geometry/tool defaults and is loaded at startup.
No collision checking is included (the original collision/visualization modules
are deliberately not imported). IK success means only pose tolerances and
joint limits were satisfied, not collision clearance or trajectory feasibility.

## Legacy base frame versus ROS base

The solver base is explicitly **`arm_base_legacy`**, with Y up. It is not a
REP-103 Z-up base. A `base_frame` configuration name must end in `_legacy`;
changing a name never converts coordinates. Both services use this configured
base and TCP; `Pose` has no frame field, so callers must transform inputs first.

For coincident origins and matching forward X, the coordinate mapping to a
ROS X-forward/Y-left/Z-up base is:

```text
p_ros = [x_legacy, -z_legacy, y_legacy]
R_ros_tcp = Rx(+pi/2) @ R_legacy_tcp
T_ros_legacy: translation [0,0,0], quaternion_xyzw [sqrt(0.5),0,0,sqrt(0.5)]
```

This defines an axis convention, not a measured mounting transform. The node
does not publish that static transform: bringup must establish the actual base
placement. See the official [REP-103 convention](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst).

## ROS integration

Depends on the shared **`urc_interfaces`** package; does not generate duplicate
interfaces. Expected definitions are exactly:

```text
# SolveIK.srv
geometry_msgs/Pose target
float64[6] seed
bool require_orientation
float64 max_joint_delta_rad
---
bool success
string message
float64[6] positions
float64 position_error_m
float64 orientation_error_rad
bool orientation_relaxed
geometry_msgs/Pose achieved_pose

# ComputeFK.srv
float64[6] positions
---
geometry_msgs/Pose pose
```

From a sourced ROS 2 workspace, build with `colcon build --packages-up-to
urc_kinematics`, source `install/setup.bash`, and run:

```sh
ros2 run urc_kinematics kinematics_node
ros2 run urc_kinematics kinematics_node --ros-args -p config_path:=/absolute/path/model.json
```

Node name: `urc_kinematics`. Relative service names: `solve_ik`, `compute_fk`
(therefore `/solve_ik`, `/compute_fk` without a namespace). Startup-only parameters:

| Parameter | Default | Purpose |
|---|---|---|
| `config_path` | Installed `config/demo.json` | Explicit SI model, limits, tool and solver settings |
| `publish_tcp_tf` | `false` | Enable measured TCP and J5-output TF |
| `joint_states_topic` | `/joint_states` | Measured input, sensor-data QoS |
| `joint_names` | `joint_1` ... `joint_6` | Six distinct names in J1..J6 order |
| `joint5_frame` | `arm_joint5_output` | J5 post-rotation frame at J5 center |

TF uses the message timestamp, requires all six named joints in one message,
reorders by name, and rejects duplicate/missing/out-of-limit/nonfinite data.
It publishes two sibling transforms under the configured legacy base: the TCP
and J5 output. J5 output is at the J5 center after J5 rotation, **before L5**;
J6/tool changes do not move that frame. No camera frame/extrinsics, URDF, or
commanded-state TF is fabricated. Subscribers should enforce freshness of the
measurement timestamp; the node does not extrapolate or combine partial states.
Avoid enabling this broadcaster alongside another broadcaster of these frames.

IK requires a finite in-limit seed; invalid seeds are rejected rather than
clipped. Unit quaternions tolerate norm error up to 1e-6, then normalize.
Both pose errors are recomputed from FK before accepting a result. It first
tries the seed, then bounded numerical refinement and analytic geometry seeds
ordered near the supplied seed, with deterministic fallback starts. It does
not guarantee the globally nearest branch or a continuous trajectory.

A valid but unsolved request returns `success=false`, a rejected in-limit
candidate, and its actual errors. Invalid IK requests return `success=false`,
NaN positions, infinite errors and a diagnostic message. **ComputeFK has no
error fields in its contract:** invalid inputs log an error and return a pose
whose seven components are NaN. Clients must reject nonfinite FK responses.
Solver iteration budgets are finite but are not a real-time deadline. A failed
numerical solve does not prove unreachability; the outer reach bound does.

## Local tests without ROS

Install Python, NumPy and SciPy (pytest optional), then from this package:

```sh
python -m unittest discover -s test -v
# Or:
python -m pytest test -q
```

Tests check independent homogeneous-matrix FK against the original geometry,
SI units, rotated/translated tools, full-pose round trips, seed preservation,
wrist singularities, bounds, invalid inputs, unreachable targets, J5 semantics,
JointState mapping and service error responses using message-shaped objects.
ROS-generated messages, DDS services and TF transport require a sourced ROS
installation and the shared interfaces; pure tests do not claim to test those.
