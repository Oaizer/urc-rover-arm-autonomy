# URC autonomous typing project — working notes

Updated 2026-09-21. This document describes the code and tests currently in the workspace. It is not a claim that the rover can type on hardware yet.

## Goal and hardware layout

The rover is parked manually in front of the Equipment Servicing keyboard. The Pi 5 surveys the keyboard, determines its pose relative to the arm base, and types a 3–6-character launch key with a rigid stick. The laptop is for video, status, launch-key entry, Start, and Abort; camera processing and mission decisions belong on the Pi.

Planned hardware:

- Raspberry Pi 5, direct Ethernet connection to a Windows laptop, USB-C power.
- Wrist-area USB camera, rigidly mounted at J5. Whether the bracket rotates with J5 still needs to be checked physically.
- Six-axis arm using the existing Y–Z–Z–X–Z–X IK model.
- CubeMars motors driven by moteus-x1 controllers over CAN-FD, with MA600 output-angle sensing. Motor models, reductions, controller IDs, electrical limits, and commutation sensing have not been recorded or validated.
- A non-actuated stick for pressing keys.
- Redragon K552 keyboard and nominally 20 mm ArUco squares. The intended tag IDs are 0–4. Their actual positions and the printed marker dictionary need to be confirmed on the practice/competition setup.

The existing IK model uses millimetres and radians. Its current offsets are `d1=50`, `L2=350`, `L3x=50`, `L3y=100`, `L4=300`, `L5=30`, `L6=100` mm. These are values in the software model, not a completed measurement of the assembled arm. The IK project is at `C:\Users\oaize\Documents\Codex\2026-09-19\i-need-to-design-an-inverse` on this computer.

## Current control concept

Typing is split into two phases because the camera may not see all tags from one pose.

1. **Survey.** The arm remains in front of the keyboard and visits safe center/left/right/up/down observation poses. At each settled pose, the camera detects whichever tags are visible. Encoder readings bracket the image capture; joint angles are interpolated to the image timestamp. A tag pose is transformed into the arm-base frame using `T_base_J5 × T_J5_camera × T_camera_tag`. Repeated detections are fused by ID. The survey stops when enough distinct, non-collinear IDs, observations, viewpoint separation, and geometric consistency are available.
2. **Type.** Known tag-center coordinates in the keyboard frame are registered against the fused 3D tag positions. This yields `T_base_keyboard`. Key centers are transformed into base coordinates, passed to IK, then approached, pressed, retracted, and checked against display readback.

Three non-collinear tag centers define a plane. They do **not** locate individual keys unless the ID-to-keyboard coordinates are measured or the keyboard outline/key grid is independently registered. The present implementation assumes a measured tag layout will be supplied.

## What is in the workspace

`outputs/urc-keyboard-autonomy/` contains the mission prototype:

| File | Role |
| --- | --- |
| `config.json` | Live configuration template; calibration fields deliberately blank. |
| `config.starter.json` | Populated **synthetic** values for software testing only. It is not a measured configuration. |
| `run_starter_test.ps1` | Windows one-command validation, unit tests, and simulated `QAZ` run. |
| `validate_setup.py` | Reports simulation readiness and live-mode configuration blockers. It checks presence/shape, not whether values were physically measured. |
| `run_keyboard_mission.py` | CLI entry point for simulation and guarded live path. |
| `perception.py` | USB/V4L2 camera capture, individual ArUco pose estimation, older four-tag localizer, basic OCR wrapper. |
| `survey.py` | Front-viewpoint executor, timestamp interpolation, multi-view tag fusion, plane fit, keyboard registration, confidence checks. |
| `mission.py` | Typing state machine, approach/press/retract, display comparison, basic backspace correction, fault/timeout stops. |
| `ik_adapter.py` | Calls the separate six-axis IK project and computes the J5 camera frame. |
| `hardware.py` | Simulated joints and moteus bridge; controller-to-output-joint conversion. |
| `interfaces.py` | Data structures for feedback, detections, survey results, and mission states. |
| `simulate_active_survey.py` | Synthetic partial-visibility survey. |
| `offline_test.py`, `stress_test.py`, `test_*.py` | Synthetic vision tests, stress scenes, and unit tests. |
| `CALIBRATION.md` | Physical calibration sequence. |
| `arm_motion_demo.py` | Separate slow IK animation; not a closed-loop hardware test. |

`outputs/pi-aruco-low-latency/` contains a **separate** video-preview prototype. `pi_sender.py` detects and annotates tags on the Pi and sends JPEG packets over UDP. `laptop_receiver.py` reassembles frames, keeps a small bounded jitter buffer per camera, and serves a browser page at port 8080. The sender supports multiple USB/V4L2 cameras, assigning `cam/0`, `cam/1`, etc. This preview code is not yet wired into the typing mission. The preview buffer must never be used as the source of arm-control observations.

The interactive browser simulator previously served on port 8765 was a visual sketch, not the current survey algorithm or real IK. Do not use its displayed latency numbers as benchmarks.

## Run the software test on this Windows computer

From PowerShell:

```powershell
cd "C:\Users\oaize\Documents\Codex\2026-09-20\i-have-a-new-project-i\outputs\urc-keyboard-autonomy"
powershell -ExecutionPolicy Bypass -File .\run_starter_test.ps1
```

The script checks `config.starter.json`, runs all `test_*.py` files, and runs the simulated survey and `QAZ` typing flow. Individual commands:

```powershell
python validate_setup.py --config config.starter.json
python -m unittest discover -s . -p "test_*.py" -v
python simulate_active_survey.py
python run_keyboard_mission.py --config config.starter.json --simulate --key QAZ
python offline_test.py
python stress_test.py
python arm_motion_demo.py
```

On 2026-09-21, the ten unit tests passed. The simulated survey saw ID 4 at center, IDs 0 and 3 from the left, then IDs 1 and 2 from the right; it completed after three viewpoints. The simulated typing state machine reported `QAZ` complete. The synthetic survey previously reported 0.303 mm plane RMS, 0.383 mm registration RMS, and 0.129 mm translation error against its generated ground truth. These figures come from idealized geometry and small injected noise, not a physical camera or arm.

The starter profile has `calibration_status=SIMULATION_ONLY` and `torque_limit=0`. Running `--live` with it is refused. Do not change either field just to get past the guard.

## Pi and laptop preview operation

For a direct Ethernet cable with no router, the intended static pair is laptop `192.168.50.1/24` and Pi `192.168.50.2/24`. Verify link lights, assigned addresses, and ping before debugging the stream. On Raspberry Pi OS, use NetworkManager/`nmtui` for a persistent wired address; `sudo ip addr add 192.168.50.2/24 dev eth0` is temporary. The Pi’s USB-C power connector is not the Ethernet data path.

USB cameras appear as `/dev/videoN`. `rpicam-hello --list-cameras` is for CSI/libcamera cameras and can report no cameras even when USB webcams are connected. Check:

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
v4l2-ctl --list-formats-ext -d /dev/video0
```

One physical USB camera can create several video nodes; choose its capture node, preferably through a stable `/dev/v4l/by-id/...-video-index0` path. On the Pi, after copying `pi_sender.py`:

```bash
python3 pi_sender.py --host 192.168.50.1 --port 5000 \
  --camera-devices /dev/video0,/dev/video2 \
  --width 640 --height 480 --fps 15 --detect-width 480 --detect-every 2
```

On the laptop, from the directory containing `laptop_receiver.py`:

```powershell
py laptop_receiver.py --bind 0.0.0.0 --port 5000 --cameras 2 --http-port 8080 --buffer-frames 2
```

Open `http://127.0.0.1:8080/`. The URL is local to the laptop; the Pi sends UDP to the laptop’s Ethernet address. For one camera, pass one device and use `--cameras 1`. Lower preview resolution, frame rate, JPEG quality, or detection frequency if the Pi is overloaded. Do not increase the jitter buffer to hide a detection problem.

To copy a file from Windows PowerShell to the Pi, substitute the real username and address without angle-bracket placeholders. The Pi account previously used the name `ubuntu`:

```powershell
scp "C:\path\to\pi_sender.py" ubuntu@192.168.50.2:/home/ubuntu/pi_sender.py
```

## Lessons from setup so far

- The original Pi had boot/power interruptions and login confusion. A login prompt eventually appeared, but the exact cause of the earlier red/yellow power indication and keyboard freeze was **not established**. A charger advertising 64 W is not proof that the Pi receives the correct negotiated voltage/current; verify the supply, cable, and Pi power warnings before arm work.
- `RTNETLINK answers: Operation not permitted` came from trying to change networking without root privileges. Use `sudo` for `ip link`/`ip addr` on the Pi. The correct interface shown in the earlier terminal was `eth0`, not `etho`.
- PowerShell treats `<pi-user>` as syntax, not a placeholder. Use the actual username (`ubuntu`) in `scp`.
- The first camera sender assumed Picamera2/CSI and failed when its requested `FrameDurationLimits` control was not advertised. The cameras are USB; the V4L2 backend is the correct starting point. Listing cameras through `rpicam` does not enumerate USB webcams.
- The first visual test used front-facing, fully visible tags and was too easy. Perspective/noise/occlusion tests and then a multi-view survey were added. A synthetic success must not be interpreted as a measured targeting error.
- Early code placed the camera at the tool/TCP frame. The planned mount is at J5, so `T_base_J5 × T_J5_camera` is now modeled separately from the stick-tip transform.
- One observation is not enough in poor visibility. Partial views are accumulated by marker ID, and the transition to typing requires non-collinear geometry and multiple viewpoints.
- Marker size is the black square, not its white quiet-zone margin. The keyboard markers are planned as 20 mm; the 10 mm markers discussed earlier were for a different task.

## What has not been demonstrated

No physical Pi/arm/camera/tag trial has been run for the current two-phase mission. In particular:

- The simulated mission uses a stub IK solver that returns six zero angles and a simulated display verifier. Its `COMPLETE` result proves software handoff and state progression, **not** key reachability, safe motion, contact, or OCR accuracy.
- The separate arm animation calls the real IK model but uses a chosen target, not a measured keyboard. It does not show the active survey or a full typing attempt.
- `config.starter.json` contains guessed camera intrinsics, zero distortion, tag/key coordinates, J5 camera offset, controller ratios/signs, joint limits, and joint-angle survey poses. None should be copied into a live profile as measured values.
- The current code fits a plane and rigidly registers tag centers. It does not yet validate the keyboard outline/key grid if tags are moved independently of the keyboard. It also does not re-survey before every press.
- The mission currently reads the display once after each press. Stable repeated OCR readings, refresh-delay handling, and robust correction of missing/extra characters remain to be implemented and tested.
- `press_speed_mm_s` is currently passed into a joint velocity limit in rad/s. This unit mismatch must be fixed before any contact motion. The current approach/contact commands are joint endpoints, not a validated straight-line tool path with force/depth limits.
- The configured `T_wrist_tip` is required by the live configuration guard but is not yet applied by the IK typing target. The stick-tip location and orientation therefore cannot be trusted in live motion.
- Collision-free paths, gravity-loss response, watchdog behavior, actual moteus/MA600 register mapping, electrical limits, and safe stop under CAN loss are not commissioned.
- Camera frames are timestamped after `VideoCapture.read()`. That is not necessarily exposure time; the stated 10 ms image/encoder synchronization threshold needs hardware validation or a better camera timestamp source.
- The preview stream and mission capture are separate programs. USB bandwidth, camera ownership, and Pi CPU load with both enabled have not been benchmarked.
- Real hardware end-to-end streaming, keyboard localization, and autonomous typing have not been shown. A browser page at port 8080 can be open without valid camera frames arriving.

## Next work, in order

1. **Freeze hardware details.** Record actual camera device and formats, physical J5 mounting side, CubeMars models, gearboxes, moteus IDs/firmware, MA600 wiring, supply, and emergency-stop behavior. Verify the Pi and direct Ethernet link separately.
2. **Measure transforms and geometry.** Confirm arm dimensions and joint zeros/signs/limits. Calibrate camera intrinsics at operating resolution/focus. Measure `T_j5_camera` and `T_wrist_tip`. Measure the actual tag IDs/centers and K552 key centers/press travel. Replace synthetic configuration values with measured ones in a separate live profile.
3. **Close the motion-safety gaps.** Apply the stick-tip transform to IK; implement time-parameterized joint/tool trajectories with consistent units, collision checking, torque/contact limits, tracking and watchdog checks, and a tested response to power/CAN loss. Commission one joint at a time with no keyboard present.
4. **Perception-only trials.** Hold the arm unpowered or use safe observation poses; collect real images from multiple angles. Check per-tag pose error, inter-view consistency, plane fit, keyboard-frame accuracy, and actual timestamp mismatch. Require independent key-center targeting measurements before contact.
5. **Non-contact arm trials.** Commission front-only viewpoints individually, then hover over all planned keys and Backspace. Confirm actual stick-tip position and clearances with external measurement.
6. **Contact and readback trials.** Use a practice keyboard, establish a safe press-depth window, add fresh localization before each press and stable OCR readback after it, then inject camera/CAN/encoder/OCR failures.
7. **Full attempts and load tests.** Run randomized 3–6-letter strings, repeated letters, distant transitions, and error recovery. Benchmark while preview streaming; reduce preview load before reducing control-image quality.

The live guard is a configuration gate, not a safety certification. Do not mark calibration as verified or assign a positive torque limit until the hardware and software checks above are complete.
