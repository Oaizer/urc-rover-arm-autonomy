# URC rover arm project

This repository preserves the ROS 2 arm testbed and the earlier Raspberry Pi and
autonomous typing prototypes. The ROS 2 simulation is the current integration
path; the earlier scripts remain available as references and standalone tests.

| Directory | Contents |
| --- | --- |
| [`ros2_ws/`](ros2_ws/README.md) | ROS 2 Jazzy packages, simulated camera and arm, tag mapping, IK, mission actions, RViz, and tests |
| [`outputs/urc-keyboard-autonomy/`](outputs/urc-keyboard-autonomy/README.md) | Earlier standalone keyboard typing and survey prototype |
| [`outputs/pi-aruco-low-latency/`](outputs/pi-aruco-low-latency/README.md) | Pi camera sender and laptop video receiver |
| [`outputs/PROJECT_GUIDE.md`](outputs/PROJECT_GUIDE.md) | Project history and operating notes |
| [`work/tag-ik-simulator.html`](work/tag-ik-simulator.html) | Earlier interactive browser simulation |
| [`work/urc2027-rules.pdf`](work/urc2027-rules.pdf) | Local copy of competition rules used for planning |

## Run the ROS 2 simulation

Use Ubuntu 24.04 with ROS 2 Jazzy. Follow the dependency and build instructions
in [`ros2_ws/README.md`](ros2_ws/README.md). In the existing Windows/WSL setup,
`ros2_ws/start_sim.ps1` builds and opens RViz. The full simulated mission and
ROS interfaces are described in [`ros2_ws/AUTONOMY.md`](ros2_ws/AUTONOMY.md).

The simulated mission finds a synthetic tag panel and visits three hover targets.
Motor commands, collision checks, grasping, lock insertion and physical mission
validation are still to be implemented. The real CAN node currently reads
feedback only. The older keyboard prototype uses an IK path outside this
repository; update its configuration if running it on another machine.

Generated ROS build/install/log directories, Python caches, screenshots and local
telemetry are intentionally excluded from version control.
