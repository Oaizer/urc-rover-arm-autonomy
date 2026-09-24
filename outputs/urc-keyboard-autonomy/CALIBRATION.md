# Commissioning and calibration procedure

Do these steps with motor power disabled until the software reports valid
feedback and all six joint limits are entered.

## Joint and MA600 calibration

For each moteus-x1:

1. Record the controller ID, motor part number, gearbox ratio, commutation
   sensor, and MA600 connection port.
2. Configure the MA600 as the output source using the moteus encoder guide.
3. Move the joint by hand through its usable range and verify that the output
   angle sign agrees with the IK positive direction.
4. Record the zero offset at the mechanical zero mark.
5. Compare output angle change against the measured joint angle, not the motor
   rotor angle.
6. Enter the result in `config.json` and repeat until the six-joint FK pose
   matches a measured pose.

The controller must retain a separate commutation source when the MA600 is an
output sensor. Do not enable motion until the sign, offset, and ratio are
verified for every joint.

## Camera calibration

Use the same resolution, focus, and lens state used during typing. Capture at
least 20 views of a rigid calibration board across the complete wrist-camera
image area. Save the camera matrix and distortion coefficients in `config.json`.

Then perform eye-in-hand calibration by holding a fixed target while moving
the arm through diverse orientations. Because this camera is mounted at J5,
save `T_j5_camera` as a 4x4 homogeneous transform in millimeters. Record
whether the bracket follows the rotating J5 output or is fixed to the upstream
J5 housing in `camera_rotates_with_j5`.

Verify this transform independently: observe one stationary tag from at least
five arm poses and transform every observation into the base frame. The fused
3D tag positions should agree within 2 mm before keyboard surveying is enabled.

## Keyboard registration

Measure the actual Redragon K552 key centers in a keyboard coordinate frame.
Use the top-left key-center origin and make the keyboard plane `z=0`; set `+z`
outward from the keyboard toward the typing stick. Enter the measured centers
for tag IDs 0 through 4 in `keyboard.tag_centers_mm`. The survey requires at
least three non-collinear IDs and uses additional IDs for validation.

The black ArUco square's physical size is 20 mm for the keyboard task. Measure
the offset from each black-square center to the keyboard frame; do not use the
white border as the marker size.

## Stick calibration

With the arm at a known pose, measure the stick tip relative to the calibrated
wrist frame and save `T_wrist_tip`. Validate the transform by hovering above
several key centers. The first contact test should use a paper target or a
force-limited mock key before the real keyboard.
