# URC 2026 autonomous keyboard typing

This is the first Pi-side implementation slice for the URC 2026 Equipment
Servicing typing task. It keeps perception, IK, mission decisions, OCR, and
hardware supervision on the Raspberry Pi. The laptop remains a low-latency
video/status viewer.

The code is intentionally conservative around hardware: the exact CubeMars
motor model, reduction, CAN IDs, camera device, MA600 configuration, marker
IDs, and measured keyboard layout must be filled into `config.json` before
enabling motion.

## Install on the Pi

```bash
sudo apt update
sudo apt install -y python3-opencv python3-scipy python3-pytesseract v4l-utils
python3 -m venv --system-site-packages ~/urc-venv
source ~/urc-venv/bin/activate
python -m pip install moteus numpy
```

Copy this directory and the existing IK project to the Pi. Set
`ik_project` in `config.json` to the Pi path of the IK project.

## Run the safe simulation

```bash
python3 run_keyboard_mission.py --config config.json --simulate
```

Run the generated four-tag perception test without any camera or tags:

```bash
python3 offline_test.py
```

It checks the real ArUco detector, PnP pose estimation, marker association,
and reprojection-error threshold using a physically consistent synthetic scene.

Run the harder perspective stress test:

```bash
python3 stress_test.py
```

It renders many keyboard views with camera roll/pitch/yaw, distance, exposure,
noise, blur, and interior occlusion. It writes `stress_test_montage.png` and
`stress_test_results.json`. The result is diagnostic: failures identify camera
mounting and vision conditions that must be handled before hardware testing.

The live command requires all calibration and hardware fields to be marked
valid in the config:

```bash
python3 run_keyboard_mission.py --config config.json --live
```

The live path refuses to move if the camera pose, tag layout, keyboard model,
joint limits, CAN IDs, or safety limits are missing.

## Configuration workflow

1. Fill in the six moteus IDs, motor/output ratios, joint limits, and calibrated
   zero offsets.
2. Configure each MA600 on its moteus x1 and verify output encoder sign and
   offset while moving the joint by hand.
3. Fill in the camera path, tag dictionary, marker IDs, and measured keyboard
   tag centers in the keyboard frame.
4. Run the camera calibration and eye-in-hand calibration procedure, then save
   `camera_matrix`, `distortion`, and `T_wrist_camera`.
5. Measure all K552 key centers and the stick-tip transform.
6. Run non-contact hover tests before enabling `PRESS`.

The perception path is now split into two phases. `ActiveKeyboardSurvey` moves
through front-only observation viewpoints, estimates each visible marker in the
camera frame, transforms it through the timestamped J5 pose into the arm-base
frame, fuses repeat sightings, fits the keyboard plane, and registers the known
tag layout. `KeyboardMission` accepts the resulting `T_base_keyboard` and then
follows `PLAN -> APPROACH -> PRESS -> RETRACT -> VERIFY`.

Run the complete poor-visibility software simulation:

```bash
python3 run_keyboard_mission.py --config config.json --simulate --key QAZ
```

For an immediately runnable profile with populated synthetic parameters, use:

```bash
python3 validate_setup.py --config config.starter.json
python3 run_keyboard_mission.py --config config.starter.json --simulate --key QAZ
```

`config.starter.json` contains plausible values solely for software testing.
Its `calibration_status` and zero torque limit keep live mode locked. Never
change the status to `MEASURED_AND_VERIFIED` until every camera transform,
motor conversion, joint limit, tag/key coordinate, and survey pose has been
measured and independently checked on the physical arm.

In the simulated survey, no single view contains enough tags. The center, left,
and right observations are accumulated until the estimator has five IDs and a
confidence-gated keyboard transform. To exercise Phase 1 by itself and write
`active_survey_result.json`:

```bash
python3 simulate_active_survey.py
```

The survey rejects unexpected IDs, small detections, reprojection outliers,
stale joint/image timestamps, collinear tag sets, insufficient viewpoint
separation, excessive plane residual, and excessive registration uncertainty.
The configured IDs are `0` through `4`; at least three non-collinear IDs are
required.

The camera is modeled at J5, not at the tool tip. Fill `arm.T_j5_camera` and set
`arm.camera_rotates_with_j5` according to whether the mount is attached to the
J5 rotating output or its upstream housing. The typing stick continues to use
its separate `T_wrist_tip` calibration.

Every entry in `survey.viewpoints` has a `joint_positions_deg` field. It is
deliberately `null`: commission these six-angle poses individually with the
keyboard absent, collision checking enabled, and conservative torque limits.
Live mode refuses to start until all viewpoint poses and camera calibration
fields are populated. During a survey, the executor brackets each image with
joint feedback, interpolates the encoder state to the image timestamp, and
stops visiting viewpoints as soon as the pose confidence gates pass.

Render a slow simulated arm move from the zero pose to a reachable target:

```bash
python3 arm_motion_demo.py
```

The default target is `(150, 70, 300)` mm. Use `--target X Y Z` for another
target. The script writes `arm_motion_demo.gif` and a final PNG frame.
