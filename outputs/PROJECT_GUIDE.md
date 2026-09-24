# Getting the rover to type

The goal is to park the rover in front of a Redragon K552 keyboard, have the Pi find it, and type a 3–6-character key with a rigid stick. The laptop shows the video and will eventually provide Start and Abort. The Pi does the vision, planning, and control.

## How it works

We split the job in two because the camera may not see every ArUco tag at once.

1. **Look around.** The arm stays in front of the keyboard and moves its J5-mounted camera left, right, up, and down. Each tag sighting is converted into a 3D position relative to the arm base using the joint angles and camera calibration. Sightings from different views are combined.
2. **Type.** After seeing at least three different, non-collinear tags, the software fits the keyboard plane and places the keyboard in the arm's coordinate system. It then calculates key targets, solves IK, presses with the stick, and reads the display.

Three tags define a plane, but not where the keys are. The tag IDs and their positions relative to the keyboard must be measured.

## Try the software now

No Pi or arm is needed for this test. In PowerShell:

```powershell
cd "C:\Users\oaize\Documents\Codex\2026-09-20\i-have-a-new-project-i\outputs\urc-keyboard-autonomy"
powershell -ExecutionPolicy Bypass -File .\run_starter_test.ps1
```

This checks the starter configuration, runs ten unit tests, and simulates a partial-visibility survey followed by typing `QAZ`. The starter configuration is **simulation only**: it has a zero torque limit and live motion is locked.

The code is in `outputs/urc-keyboard-autonomy/`. The separate `outputs/pi-aruco-low-latency/` folder streams annotated USB-camera video from the Pi to a laptop browser at `http://127.0.0.1:8080/`. That preview is not connected to arm control.

## What we learned

- The cameras are USB. Check them with `v4l2-ctl --list-devices`, not `rpicam-hello --list-cameras`.
- The camera is mounted at J5, not at the typing stick. Those two transforms need separate calibration.
- A straight-on image with every tag visible was too easy a test; the current survey accumulates partial views.
- Use the actual Pi username (`ubuntu`), not `<pi-user>`, in PowerShell `scp` commands. Use `sudo` when changing the Pi's network address.
- The earlier Pi power/boot instability was not diagnosed. Confirm a stable supply and Ethernet link before motor testing.

## What is not working yet

The simulated `COMPLETE` result is **not a physical typing result**. The full mission simulation uses stub IK and a fake display. No current two-phase attempt has been run on the real arm.

Before contact, the code still needs the measured stick-tip transform applied to IK, a fix for a press-speed unit mismatch (mm/s versus rad/s), collision-checked press paths, fresh localization before each key, stable OCR readback, and a tested response to CAN/power loss. The camera, tag, key, motor, and survey-pose numbers in `config.starter.json` are placeholders.

## What to do next

Measure the real arm and camera; verify each MA600/motor sign and limit; measure the tag and key positions; then test the survey with the arm moving **without contact**. The next milestone is a repeatable hover over every needed key and Backspace with an independently measured tip-position error. Only then start press-depth trials on a practice keyboard.

For file-by-file details and the full commissioning checklist, see [PROJECT_HANDOFF.md](PROJECT_HANDOFF.md) and `outputs/urc-keyboard-autonomy/CALIBRATION.md`.
