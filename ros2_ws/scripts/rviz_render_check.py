"""Render RViz on a private Xvfb display for repeatable visual QA.

Run with xvfb-run. This opens no Windows desktop window and controls only the
temporary RViz process it owns; the existing ROS simulation supplies the data.
"""
import re
import signal
import subprocess
import sys
import time
from PyQt5.QtWidgets import QApplication

process = subprocess.Popen(['rviz2', '-d', sys.argv[1], '--ros-args', '-r', '__node:=rviz_render_check', '-p', 'use_sim_time:=true'])
try:
    window = None
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline and process.poll() is None:
        tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
        match = re.search(r'(0x[0-9a-f]+) .* - RViz"', tree)
        if match:
            window = int(match[1], 16)
            break
        time.sleep(.25)
    if window is None:
        raise RuntimeError('RViz did not create a window')
    time.sleep(10)  # bounded wait for discovery, TF and image subscriptions
    app = QApplication([])
    pixmap = app.primaryScreen().grabWindow(window)
    if pixmap.isNull() or not pixmap.save(sys.argv[2]):
        raise RuntimeError('RViz screenshot failed')
finally:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)
