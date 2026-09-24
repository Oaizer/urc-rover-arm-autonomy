# USB camera and ArUco perception

ROS 2 `ament_python` package `urc_perception`. It depends on the shared sibling
`urc_interfaces` package supplied by the workspace; this directory does not generate
or vendor messages. All package files and pure tests live in this directory.
No TF is published, and there is no assumed robot/J5 transform, competition ID,
dictionary, marker size, or camera calibration.

## Build and run (Ubuntu 24.04 / ROS Jazzy)

From `ros2_ws`, with ROS sourced:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to urc_perception --symlink-install
source install/setup.bash
ros2 launch urc_perception usb_aruco.launch.py dictionary:=DICT_4X4_50
```

The dictionary above is an explicit example choice, not a URC requirement. Nodes
start in namespace `camera`; use `namespace:=...` to change it. `device` defaults
to `/dev/video0`; a stable `/dev/v4l/by-id/...` path is preferable. On WSL, a real
camera requires USB passthrough and a working V4L2 device; synthetic ROS integration
can run without hardware. Device permissions must allow opening the camera.

Launch arguments: `dictionary` (required), `namespace`, `device`,
`calibration_file`, `params_file`. Supply one calibration file to both nodes:

```bash
ros2 launch urc_perception usb_aruco.launch.py dictionary:=DICT_4X4_50 \
  calibration_file:=/absolute/path/camera.yaml \
  params_file:=/absolute/path/usb_aruco.yaml
```

Copy/adapt `config/usb_aruco.yaml`; add e.g. `tag_sizes: ['7:0.15', '12:0.20']`
under the detector's `ros__parameters`. Those IDs and sizes are examples only.
Measure the outer black square, excluding the white margin, in metres. Every
detected ID is reported, but only IDs with configured sizes can receive metric poses.
Launch's explicit device/dictionary/calibration arguments override their YAML values.

Independent entrypoints (relative topics are remappable):

```bash
ros2 run urc_perception usb_camera --ros-args -p device:=/dev/video0
ros2 run urc_perception aruco_detector --ros-args -p dictionary:=DICT_4X4_50
```

Use distro OpenCV and `cv_bridge` together on ROS. OpenCV 4.6's
`DetectorParameters_create` / `detectMarkers` are supported as well as the newer
`DetectorParameters` / `ArucoDetector` API. Do not install the Windows test pip
dependencies over the ROS system OpenCV/NumPy environment.

## Parameters and topics

All application parameters are startup-only (read-only after declaration).

| Node | Parameter | Default / meaning |
|---|---|---|
| camera | `device` | `/dev/video0`, string V4L2 device path |
| camera | `width`, `height` | `640`, `480`, requested acquisition dimensions |
| camera | `capture_fps`, `publish_fps` | `30.0`, `30.0`; acquisition vs publication limit |
| camera | `fourcc` | `MJPG`, four-character codec request |
| camera | `frame_id` | `camera_optical_frame` |
| both | `calibration_file` | empty = uncalibrated; absolute ROS camera YAML path |
| detector | `dictionary` | empty; must explicitly set a supported `DICT_*` name |
| detector | `tag_sizes` | unset/empty string array = no metric poses; entries `ID:metres` |
| detector | `detection_fps` | `30.0`, processing limit |
| detector | `max_image_age_s` | `0.25`, strictly positive finite maximum receipt age |
| detector | `ambiguity_policy` | `reject`; alternatively `flag` |
| detector | `ambiguity_gap_px` | `0.2`, absolute difference between hypothesis errors |
| detector | `ambiguity_ratio` | `1.5`, second/best error threshold (>=1) |
| detector | `max_reprojection_error_px` | `3.0`, maximum best RMS corner error |
| detector | `publish_annotated` | `false` |
| detector | `annotated_fps` | `5.0`, optional annotation limit |

Camera publishes `image_raw` (`sensor_msgs/Image`, BGR8) and `camera_info`
(`sensor_msgs/CameraInfo`). Detector subscribes to `image_raw` and publishes
`tag_detections` (`urc_interfaces/TagDetections`) and optionally `image_annotated`.
Every publisher/subscription uses sensor QoS: BEST_EFFORT, VOLATILE, KEEP_LAST,
depth 1. Consumers must request compatible QoS (a RELIABLE subscription will not
match). Image and CameraInfo have identical stamps/frame IDs and actual image
dimensions; best-effort transport can still drop either message independently.

## Timing and bounded buffering

A dedicated reader continuously calls V4L2 `read()` and overwrites one latest
frame slot. The ROS timer atomically consumes that slot at most once per frame;
it cannot build up a queue or repeatedly publish an old frame. The driver buffer
size is requested as one, but some devices ignore it. Continuous draining reduces
backlog; it cannot guarantee a maximum USB/driver/decode delay. The detector also
has a single-slot mailbox and depth-1 subscription.

The camera stamps immediately **after `read()` returns decoded pixels**, using
the node's ROS clock. This is user-space receipt time, **not sensor exposure time**
and not the kernel V4L2 buffer timestamp. USB, driver queue, read, and JPEG decode
latency precede it. Do not interpret this stamp as exposure synchronization. Hardware
exposure timing requires a different acquisition backend that exposes reliable
hardware timestamps and a defined clock conversion.

The detector preserves the input header. Zero, negative, future, and older-than-
`max_image_age_s` stamps cause the frame to be dropped with **no detections or
annotation publication**. Age is checked before processing and again immediately
before each output publication, so processing time counts. This checks receipt age,
not physical exposure age. Nodes must share a clock domain; for ROS simulation or
bag replay configure `use_sim_time` consistently and publish `/clock`. A backward
clock jump rejects future-stamped pending frames. No stamp is rewritten to make
an old observation appear fresh. Consumers must still expire their own last result;
silence is not evidence that a previously observed tag remains present.

Thirty consecutive failed reads stop camera publication and raise an error.
Restart the camera node after fixing the device. Shutdown joins the reader for
two seconds; a hung driver read is logged and the daemon reader exits with the
process. OpenCV release happens on the reader thread, avoiding concurrent
read/release crashes. Automatic reconnect is not implemented.

## Calibration and pose semantics

Both nodes independently load the same optional calibration YAML at startup.
The detector deliberately does not subscribe to CameraInfo: this avoids treating
an unrelated/asynchronously delivered calibration as belonging to an image.
For an external camera, set the detector's calibration file to that camera's raw
intrinsics. Supported ROS calibration keys are `image_width`, `image_height`,
`camera_matrix: {data: [nine values]}`, `distortion_model`, and
`distortion_coefficients: {data: [...]}`. Supported models are `plumb_bob` (4/5
coefficients) and `rational_polynomial` (8). Fisheye/equidistant is rejected.
An empty path is intentionally uncalibrated; an explicit unreadable or malformed
file fails startup instead of silently changing mode.

Actual image dimensions must exactly match calibration dimensions. No automatic
intrinsic scaling, cropping, binning, rectification, or focal length invention is
performed. A mismatch logs an error and retains pixel detection only. CameraInfo
uses K[0]=0 when missing/mismatched calibration. For valid raw calibration, it
publishes K/D, identity R and P=[K|0]. Matching dimensions alone cannot validate
that the lens, focus, crop, or physical camera stayed the same; recalibrate after
such changes.

Corners are refined with subpixel refinement on grayscale input and reported in
decoded-marker order: top-left, top-right, bottom-right, bottom-left, pixels with
z=0. Pose maps marker coordinates into the camera optical frame (+x right, +y down,
+z forward). Marker origin is its center; +x points toward the marker's right,
+y toward its top and +z out of its printed face. Pose translation is metres and
orientation is a unit quaternion. Nothing is transformed into a robot/world frame.

`SOLVEPNP_IPPE_SQUARE` returns two hypotheses. Each is checked for finite values
and **positive depths of all four transformed corners**, then independently
reprojected with K/D. Error is RMS Euclidean pixel distance per corner. The best
candidate must pass `max_reprojection_error_px`. Two viable candidates are
ambiguous if their error gap is <= `ambiguity_gap_px` OR second error is <=
`ambiguity_ratio * max(best error, 1e-12)`. These conservative thresholds are
configurable, not a statistical uncertainty guarantee. With `reject`, ambiguous
poses have `pose_valid=false`; with `flag`, a passing best pose may have both
`pose_valid=true` and `ambiguous=true`. No temporal disambiguation is performed.

The shared schemas consumed by this package are exactly:

```text
# TagDetection
int32 id
geometry_msgs/Point32[4] corners
bool pose_valid
geometry_msgs/Pose pose
float64 reprojection_error_px
bool ambiguous

# TagDetections
std_msgs/Header header
string dictionary
TagDetection[] detections
```

When `pose_valid=false`, the default all-zero Pose is an **unavailable payload**,
not a metric estimate (its zero quaternion is deliberately not a valid rotation).
Consumers must gate all use of pose on `pose_valid`. Reprojection error is NaN
when no candidate was evaluated, or the best error when a candidate was rejected.
An empty detection list is published for a fresh processed frame with no tags.

## Tests

ROS-independent tests run directly from this directory:

```bash
python -m pytest test -q
```

For a separate non-ROS Python environment install `requirements-test.txt` first.
Tests cover synthetic marker decoding, corner order/refinement, BGR/grayscale,
missing calibration/size, resolution mismatch, distorted projected corners and
metric pose recovery, IPPE ambiguity policies, error/depth rejection, legacy API
compatibility, timestamps, latest-slot draining, and flat ROS package metadata.
`test/test_core.py`, `test/test_capture.py`, and `test/test_packaging.py` do not
import ROS. ROS build, generated messages, live QoS/stamp checks, and actual USB
timing need the Jazzy integration environment; pure tests do not validate them.

API references: [OpenCV pose estimation](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html),
[ROS QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html).
