# ros2_control arm integration

The full RViz simulation now uses the standard ROS 2 Jazzy control stack:

```text
hover_mission -> arm_controller -> /solve_ik
                       |
     /arm_trajectory_controller/follow_joint_trajectory
                       |
             joint_trajectory_controller
                       |
                controller_manager
                       |
      mock_components/GenericSystem (J1..J6)
                       |
          joint_state_broadcaster -> /joint_states -> TF/RViz/mapper
```

`urc_description.model.make_urdf(..., control_backend='mock')` adds a
`<ros2_control>` System with six position/velocity command and state interfaces.
Initial joint positions match the camera-observation pose. The mock hardware
mirrors position and velocity commands to state. The controller configuration is
[`src/urc_simulation/config/controllers.yaml`](src/urc_simulation/config/controllers.yaml).
`src/urc_simulation/launch/demo.launch.py` starts the controller manager and
spawns both controllers. The old Python joint executor is not launched.

## Run and inspect

From Ubuntu 24.04 with ROS 2 Jazzy and the workspace installed:

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/oaize/Documents/Codex/2026-09-20/i-have-a-new-project-i/ros2_ws
source install/setup.bash
bash scripts/run_demo.sh
```

In a second terminal, source the same files and use the simulation domain:

```bash
export ROS_DOMAIN_ID=71 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Expect one active mock System, six `position` and `velocity` state/command pairs,
an active `joint_state_broadcaster`, and an active `arm_trajectory_controller`.
`/joint_states` must have one publisher. If the trajectory controller is inactive,
the pose controller will reject movement because its action server is unavailable.

The existing `arm_controller` now accepts a `trajectory_action` parameter; the
simulation sets it to the standard trajectory controller action. Its external
`/arm/move_to_pose` action and the mission's Python interface are unchanged.

## Physical arm boundary

This commit exercises the control stack with **mock hardware only**. The
`urc_can` package still performs read-only queries and must not run as a second
publisher of `/joint_states` alongside the joint-state broadcaster.

A physical `hardware_interface::SystemInterface` plugin is still needed for the
six moteus controllers. It must expose the same joint interfaces, read the
measured output angles/velocities and controller faults, convert explicitly
between controller revolutions and joint radians, and write commissioned
position/velocity commands. Keep the controller manager and broadcasters;
select the physical plugin and measured limits in a separate URDF/launch
configuration. Never substitute the mock plugin into a physical launch.

Before enabling writes, measure each joint's geometry, zero, sign, limits, and
output ratio; test a single unloaded joint; characterize a safe gravity-loaded
stop on CAN loss and power loss; and verify actual bus and Pi timing. The current
URDF efforts/velocities and controller tolerances are simulation defaults.
The controller manager does not provide a physical emergency stop or guarantee
that a disabled motor holds a load.

Jazzy references: [controller manager](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html),
[hardware interfaces](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html),
[trajectory controller](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).
