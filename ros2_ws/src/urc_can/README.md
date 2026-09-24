# urc_can: read-only feedback scaffold

An `ament_python` ROS 2 package for six configurable moteus joints. This is **not
a trajectory controller**. There is no FollowJointTrajectory action, target
subscriber, ros2_control hardware plugin, or physical command interface.

## Bringup interface

```sh
ros2 run urc_can feedback
# Equivalent explicitly simulated launch:
ros2 launch urc_can simulated.launch.py
```

Node name: `urc_can`. Default mode: `simulated`, stationary synthetic positions at
the configured offsets and zero velocity. The mock does not follow joint targets.
Do not run this synthetic publisher alongside physical joint-state publishers
on the same topic without explicit remapping.

| Startup parameter | Default | Meaning |
|---|---|---|
| `mode` | `simulated` | Only `simulated` or `read_only`; all motion modes refused |
| `backend` | empty | Real mode requires `fdcanusb` or `socketcan` |
| `channel` | empty | Explicit serial path or SocketCAN interface, e.g. `can0` |
| `joint_names` | `joint_1` ... `joint_6` | Exactly six unique nonempty names |
| `can_ids` | `[1,2,3,4,5,6]` | Exactly six unique unicast IDs in 1..126 |
| `ratios` | six `1.0` values | Controller output revolutions per joint revolution |
| `signs` | six `1` values | Each +1 or -1 |
| `offsets_rad` | six `0.0` values | Joint angle when controller position is zero |
| `rate_hz` | `20.0` | Requested polling rate, positive, at most 200 |
| `cycle_timeout_s` | `0.1` | Whole six-controller batch deadline, positive, at most 5 |
| `stale_after_s` | `0.5` | Must exceed cycle timeout and polling period |

All arrays are aligned by index. All parameters are read-only after startup;
restart to apply changes. Defaults are placeholders, not commissioned robot
calibration. A sample YAML is installed at `share/urc_can/config/simulated.yaml`.

Explicit query-only examples, after checking the actual adapter and joint map:

```sh
ros2 run urc_can feedback --ros-args -p mode:=read_only \
  -p backend:=socketcan -p channel:=can0
ros2 run urc_can feedback --ros-args -p mode:=read_only \
  -p backend:=fdcanusb -p channel:=/dev/serial/by-id/YOUR_ADAPTER
```

No adapters are auto-discovered. SocketCAN needs a separately configured Linux
CAN-FD interface with appropriate arbitration/data timing, wiring and termination.
This package does not configure interfaces, controller firmware, encoders or motors.
The real Python dependencies are optional (`pip install '.[hardware]'` in a suitable
environment). A moteus release exposing `Transport`, `FdcanusbDevice` and
`PythonCanDevice` is required; older APIs are rejected explicitly, not guessed.

## Feedback and failure semantics

`/joint_states` (`sensor_msgs/JointState`) contains all six positions in radians
and velocities in rad/s. Effort is empty. It publishes only new, complete, finite,
fault-free samples; missing fields never become zero. Missing, duplicate or
unexpected IDs invalidate the whole batch. A nonzero fault register, FAULT or
TIMEOUT mode, preparation/unknown mode, or malformed data also invalidates it.
Nonzero fault values are rejected conservatively even when firmware uses them
for a limiting condition. Valid zero readings are accepted.

`/diagnostics` uses `diagnostic_msgs/DiagnosticArray` and `DiagnosticStatus`:
OK for a fresh full sample, ERROR for failure while the last good sample is still
recent, STALE when no good sample has arrived or its age exceeds the threshold.
It includes mode, source, sample age, cumulative failed cycles and failure reason.
Diagnostics continue after real polling halts. The startup log and diagnostic
hardware ID identify simulated/stationary data explicitly. JointState itself has
no standardized simulation-mode field.

One worker thread owns one persistent asyncio event loop and the connection's
entire lifecycle. Each cycle submits six `make_query()` commands in a single
`await transport.cycle(...)`, bounded with `asyncio.wait_for`. No overlapping
cycles or unbounded result queue: ROS consumes the newest immutable status.
Publishing can drop intermediate valid samples when ROS is slower than polling.
Monotonic time controls deadlines and freshness; ROS timestamps approximate host
receipt time. Controller samples are not simultaneous or hardware synchronized.

Any real cycle failure latches polling off and closes the host connection;
restart after investigating. This deliberately prevents later batches from
mistaking delayed replies after a timeout or partial cycle for fresh data.
The transport protocol does not supply transaction IDs or acquisition timestamps;
concurrent query clients/unsolicited matching replies cannot be reliably
distinguished. Use a single query owner. Async cancellation is cooperative;
a blocking driver can overrun a deadline, but stale publication is suppressed
and late completion is rejected. This is not a hard real-time deadline guarantee.

## Safety boundary and future conversion

Only register queries are emitted. There are no position, torque, mode, stop,
torque-disable, fault-clear, calibration or configuration writes, including
startup, exceptions and shutdown. Closing the connection does **not** send a
stop command. An unsolicited stop could release a gravity-loaded mechanism.
Queries do not commission the unknown motor or establish that it is safe, stopped,
homed or held. Existing controller state/watchdogs and other software remain
outside this package's control. There is no emergency-stop function here.

`Joint.future_command(position_rad, velocity_rad_s)` is a pure conversion helper
for later commissioned controller work. It does not connect to hardware, enforce
physical limits, or make motion available. With ratio R > 0, sign S = +/-1, and
joint offset O in radians:

```text
q_joint = O + S * 2*pi * position_controller_rev / R
v_joint =     S * 2*pi * velocity_controller_rev_s / R
position_controller_rev   = S * (q_joint - O) * R / (2*pi)
velocity_controller_rev_s = S * v_joint       * R / (2*pi)
```

Moteus POSITION/VELOCITY are already in configured **output-shaft** revolutions
and rev/s. R is the remaining controller-output-to-joint ratio, not automatically
the motor gearbox ratio. If moteus already reports joint output turns, use R=1;
do not apply the same gearbox ratio twice. Likewise avoid double-applying firmware
signs/offsets. No wrapping is performed. No torque conversion is claimed.
Commissioning, limits, homing, encoder validity checks, safe holding, watchdog
design, emergency handling and hardware validation belong to a future controller.

## Tests and build

Tests are in singular `test/`, runnable without ROS or hardware:

```sh
cd ros2_ws/src/urc_can
python3 -m pytest test -q
```

ROS integration tests automatically run when `rclpy` is available; otherwise they
skip. An optional installed-moteus wire-encoding test uses fake devices and
asserts that actual query frames contain reads and no writes. No test opens CAN.
The fake API intentionally has no motion or stop methods. Tests cover unit
conversions, IDs, reply validation, stale gating, failures, persistent event loop,
timeout cancellation, connection close, and real-failure latching.

In a sourced ROS Jazzy environment, build normally with
`colcon build --packages-select urc_can`, source the install setup, then run the
entrypoint above. For a build confined to this package directory:

```sh
colcon --log-base .log build --base-paths . --build-base .build --install-base .install
source .install/setup.bash
ros2 run urc_can feedback
```

## Official API evidence

Verified against upstream on 2026-09-23 (upstream `main` is moving, not a pinned
hardware-qualified release):

- [Python API](https://mjbots.github.io/moteus/reference/python/): `make_query`,
  `QueryResolution`, asynchronous `Transport.cycle`.
- [Controller source](https://github.com/mjbots/moteus/blob/main/lib/python/moteus/moteus.py):
  `make_query()` constructs only register queries. The current async convenience
  name is `query()`, not an assumed `set_query()`.
- [Transport source](https://github.com/mjbots/moteus/blob/main/lib/python/moteus/transport.py):
  `Transport(devices)`, `cycle(commands)`, `close()`.
- [fdcanusb source](https://github.com/mjbots/moteus/blob/main/lib/python/moteus/fdcanusb_device.py):
  `FdcanusbDevice(path=...)`.
- [python-can adapter source](https://github.com/mjbots/moteus/blob/main/lib/python/moteus/pythoncan_device.py):
  `PythonCanDevice` forwards bus arguments to `can.Bus`.
- [SocketCAN documentation](https://python-can.readthedocs.io/en/stable/interfaces/socketcan.html):
  Linux SocketCAN interface and CAN-FD support.
- [Protocol source](https://github.com/mjbots/moteus/blob/main/lib/python/moteus/protocol.py):
  parsed replies use `.id` and `.values`, mode/fault registers.
- [Register reference](https://mjbots.github.io/moteus/protocol/registers/):
  output-shaft position/velocity units and mode semantics.

No physical transport or motor has been exercised. Passing fake/ROS tests is not
hardware commissioning or confirmation of mechanical safety.
