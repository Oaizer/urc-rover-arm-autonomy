# Low-latency Raspberry Pi 5 ArUco stream

This project detects ArUco tags on one or more cameras on the Pi, draws their
outlines/IDs, and multiplexes annotated JPEG frames over UDP to the laptop. All
camera/vision work happens on the Pi. The laptop only reassembles packets,
applies one small bounded transport jitter buffer per camera, and lets the
browser display the Pi-rendered JPEGs.

## Important hardware check

The laptop must show the USB Ethernet adapter as **Up**. At the time this was
created, Windows reported the adapter as `Disconnected`, so no Pi test could be
performed yet. USB-C power alone is not a data path for the Pi 5 power port;
the Ethernet cable must have a live link.

For a direct cable with no router/DHCP server, use a static pair:

- Laptop Ethernet: `192.168.50.1/24`
- Pi Ethernet: `192.168.50.2/24`

If the Pi has no address yet, run this locally on the Pi (temporary until
reboot), then test from the laptop with `ping 192.168.50.2`:

```bash
sudo ip link set eth0 up
sudo ip addr add 192.168.50.2/24 dev eth0
```

For a persistent setting on Raspberry Pi OS, run `sudo nmtui`, edit the wired
connection, choose IPv4 **Manual**, set `192.168.50.2/24`, leave gateway/DNS
empty, and activate the connection.

If the Pi is already on a DHCP/router network, use its assigned address instead.

## Install

### Pi

On Raspberry Pi OS Bookworm:

```bash
sudo apt update
sudo apt install -y python3-opencv v4l-utils
```

Copy `pi_sender.py` to the Pi. Start it with the laptop's Ethernet address:

```bash
python3 pi_sender.py --host 192.168.50.1 --port 5000 \
  --camera-devices /dev/video0,/dev/video2 \
  --width 640 --height 480 --fps 15 --detect-width 480 --detect-every 2
```

USB cameras use V4L2 device paths, not `rpicam-hello`. List the devices and
their stable names with:

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
```

One USB camera can expose several `/dev/videoN` nodes. Use only nodes that
support video capture; `v4l2-ctl --list-formats-ext -d /dev/videoN` identifies
them. Prefer the stable `/dev/v4l/by-id/...-video-index0` paths when available.
Pass one path per physical camera in `--camera-devices`, comma-separated.

The sender assigns streams `cam/0`, `cam/1`, and so on in the same order as
`--camera-devices`. For a single camera, pass just one path. The old
Picamera2/CSI mode remains available with `--backend picamera2 --camera-nums 0,1`.

Useful tuning options:

- `--width 960 --height 540`: capture/stream size; lower it for less CPU and bandwidth.
- `--detect-width 640`: detection is done on a smaller grayscale image, then boxes are scaled back.
- `--detect-every 2`: detect every second frame and reuse the last result between detections.
- `--dictionary DICT_4X4_50`: must match the dictionary used to print the tags.
- `--jpeg-quality 65`: lower values reduce bandwidth and latency.

### Laptop

No Python image-processing package is required on the laptop. Run the small
standard-library receiver:

Run the receiver:

```powershell
py laptop_receiver.py --bind 0.0.0.0 --port 5000 --cameras 2 --http-port 8080 --buffer-frames 2
```

Open `http://127.0.0.1:8080/` in a browser to see both feeds. Individual feeds
are available at `/cam/0.mjpg` and `/cam/1.mjpg`. Closing the receiver stops it.

## Detection expectations

The 1 cm physical size is not itself enough to guarantee detection: each tag
must occupy enough pixels in the camera image. Start close to the camera with
good lighting and a flat, high-contrast print. If detection is intermittent,
increase `--width/--height` or `--detect-width`, rather than increasing the
jitter buffer. The default buffer is only two frames to keep latency bounded.

## Network and latency design

- UDP avoids head-of-line blocking from lost frames.
- Frames are packetized below the Ethernet MTU and reassembled by frame ID.
- Incomplete frames expire quickly and are dropped.
- The receiver drains queued packets, keeps at most a few complete frames, and
  serves the oldest complete frame once the small jitter target is met.
- The laptop never decodes, resizes, annotates, or re-encodes the image; the
  browser performs the final JPEG display decode.
- Multiple cameras share one UDP socket using a camera ID in each packet; the
  receiver keeps their jitter buffers independent.
- Detection runs on grayscale and can be skipped on alternate frames.
- The sender reports capture, encode, and send statistics; the receiver reports
  packet loss/expiry.
