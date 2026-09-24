#!/usr/bin/env python3
"""Receive multiplexed JPEG streams and serve one low-latency page per camera."""

import argparse
import http.server
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional


MAGIC_V1 = b"ARU1"
HEADER_V1 = struct.Struct("!4sIHHQ")
MAGIC_V2 = b"ARU2"
HEADER_V2 = struct.Struct("!4sBBIHHQ")


@dataclass
class Assembly:
    packet_count: int
    timestamp_ns: int
    created_ns: int
    packets: Dict[int, bytes] = field(default_factory=dict)

    def add(self, index: int, payload: bytes) -> None:
        self.packets.setdefault(index, payload)

    def complete(self) -> bool:
        return len(self.packets) == self.packet_count

    def join(self) -> bytes:
        return b"".join(self.packets[i] for i in range(self.packet_count))


class JitterBuffer:
    def __init__(self, target_frames: int, max_age_ms: int) -> None:
        self.target_frames = max(1, target_frames)
        self.max_age_ns = max_age_ms * 1_000_000
        self.assembling: Dict[int, Assembly] = {}
        self.ready: Dict[int, tuple[int, bytes]] = {}
        self.last_played = -1
        self.expired = 0
        self.dropped = 0

    def push(self, frame_id: int, packet_index: int, packet_count: int,
             timestamp_ns: int, payload: bytes, now_ns: int) -> None:
        if frame_id <= self.last_played:
            return
        entry = self.assembling.get(frame_id)
        if entry is None:
            entry = Assembly(packet_count, timestamp_ns, now_ns)
            self.assembling[frame_id] = entry
        if entry.packet_count != packet_count:
            self.assembling.pop(frame_id, None)
            return
        entry.add(packet_index, payload)
        if entry.complete():
            self.ready[frame_id] = (entry.timestamp_ns, entry.join())
            self.assembling.pop(frame_id, None)
            while len(self.ready) > self.target_frames + 2:
                self.ready.pop(min(self.ready), None)
                self.dropped += 1

    def pop_for_display(self, now_ns: int) -> Optional[tuple[int, int, bytes]]:
        for frame_id, entry in list(self.assembling.items()):
            if now_ns - entry.created_ns > self.max_age_ns:
                self.assembling.pop(frame_id, None)
                self.expired += 1
        if len(self.ready) < self.target_frames:
            return None
        frame_id = min(self.ready)
        timestamp_ns, jpeg = self.ready.pop(frame_id)
        self.last_played = frame_id
        return frame_id, timestamp_ns, jpeg


class LatestJpeg:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.jpeg = b""
        self.version = 0

    def set(self, jpeg: bytes) -> None:
        with self.lock:
            self.jpeg = jpeg
            self.version += 1

    def get(self) -> tuple[int, bytes]:
        with self.lock:
            return self.version, self.jpeg


def start_http_server(latest_by_camera: Dict[int, LatestJpeg], port: int) -> http.server.ThreadingHTTPServer:
    camera_ids = sorted(latest_by_camera)

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_args: object) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                cards = "".join(
                    f"<section><h2>Camera {camera_id}</h2>"
                    f"<img src='/cam/{camera_id}.mjpg' alt='ArUco camera {camera_id}'></section>"
                    for camera_id in camera_ids
                )
                body = ("<!doctype html><meta name=viewport content='width=device-width'>"
                        "<title>ArUco cameras</title>"
                        "<style>html,body{margin:0;background:#111;color:#ddd;font:14px sans-serif}"
                        "main{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:8px}"
                        "section{min-width:0}h2{margin:6px 8px;font-size:14px}"
                        "img{display:block;width:100%;height:auto}</style>"
                        f"<main>{cards}</main>").encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == "/stream.mjpg":
                camera_id = camera_ids[0] if camera_ids else 0
            elif path.startswith("/cam/") and path.endswith(".mjpg"):
                try:
                    camera_id = int(path[len("/cam/"):-len(".mjpg")])
                except ValueError:
                    self.send_error(404)
                    return
            else:
                self.send_error(404)
                return

            stream = latest_by_camera.get(camera_id)
            if stream is None:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_version = -1
            try:
                while True:
                    version, jpeg = stream.get()
                    if jpeg and version != last_version:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                        self.wfile.flush()
                        last_version = version
                    time.sleep(0.01)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return

    server = http.server.ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--cameras", type=int, default=2)
    parser.add_argument("--buffer-frames", type=int, default=2)
    parser.add_argument("--max-frame-age-ms", type=int, default=100)
    parser.add_argument("--http-port", type=int, default=8080)
    return parser.parse_args()


def parse_packet(data: bytes) -> Optional[tuple[int, int, int, int, int, bytes]]:
    if len(data) >= HEADER_V2.size and data[:4] == MAGIC_V2:
        _magic, camera_id, _reserved, frame_id, packet_index, packet_count, timestamp_ns = HEADER_V2.unpack_from(data)
        return camera_id, frame_id, packet_index, packet_count, timestamp_ns, data[HEADER_V2.size:]
    if len(data) >= HEADER_V1.size and data[:4] == MAGIC_V1:
        _magic, frame_id, packet_index, packet_count, timestamp_ns = HEADER_V1.unpack_from(data)
        return 0, frame_id, packet_index, packet_count, timestamp_ns, data[HEADER_V1.size:]
    return None


def main() -> None:
    args = parse_args()
    if args.cameras < 1 or args.cameras > 256:
        raise SystemExit("cameras must be 1..256")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.01)
    jitter = {i: JitterBuffer(args.buffer_frames, args.max_frame_age_ms) for i in range(args.cameras)}
    latest = {i: LatestJpeg() for i in range(args.cameras)}
    http_server = start_http_server(latest, args.http_port) if args.http_port > 0 else None
    print(f"Listening on UDP {args.bind}:{args.port}; cameras={args.cameras}; "
          f"jitter buffer={args.buffer_frames} frame(s)", flush=True)
    if http_server:
        print(f"Browser preview: http://127.0.0.1:{args.http_port}/", flush=True)
    last_stats = time.monotonic()
    packets = [0 for _ in range(args.cameras)]
    displayed = [0 for _ in range(args.cameras)]
    try:
        while True:
            drained = 0
            while drained < 512:
                try:
                    data, _addr = sock.recvfrom(65535)
                except socket.timeout:
                    break
                drained += 1
                parsed = parse_packet(data)
                if parsed is None:
                    continue
                camera_id, frame_id, packet_index, packet_count, timestamp_ns, payload = parsed
                if camera_id not in jitter or packet_count == 0 or packet_index >= packet_count:
                    continue
                jitter[camera_id].push(frame_id, packet_index, packet_count, timestamp_ns,
                                       payload, time.monotonic_ns())
                packets[camera_id] += 1

            now_ns = time.monotonic_ns()
            for camera_id, buffer in jitter.items():
                item = buffer.pop_for_display(now_ns)
                if item is not None:
                    _frame_id, _timestamp_ns, jpeg = item
                    latest[camera_id].set(jpeg)
                    displayed[camera_id] += 1

            now = time.monotonic()
            if now - last_stats >= 5.0:
                details = " ".join(
                    f"cam{i}:rx={packets[i]} displayed={displayed[i]} expired={jitter[i].expired} "
                    f"dropped={jitter[i].dropped} ready={len(jitter[i].ready)}"
                    for i in range(args.cameras)
                )
                print(details, flush=True)
                packets = [0 for _ in range(args.cameras)]
                displayed = [0 for _ in range(args.cameras)]
                last_stats = now
    finally:
        if http_server:
            http_server.shutdown()
        sock.close()


if __name__ == "__main__":
    main()
