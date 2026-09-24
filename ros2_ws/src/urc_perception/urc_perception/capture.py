"""Continuous capture independent of ROS publication and detector latency."""
from dataclasses import dataclass
from threading import Event, Lock, Thread


class LatestSlot:
    """Bounded mailbox. Producers overwrite; consumers atomically take once."""
    def __init__(self):
        self._lock = Lock()
        self._value = None

    def put(self, value):
        with self._lock:
            self._value = value

    def take(self):
        with self._lock:
            value, self._value = self._value, None
        return value


@dataclass(frozen=True)
class Frame:
    image: object
    stamp: object


class CaptureWorker:
    def __init__(self, capture, clock):
        self.capture = capture
        self.clock = clock
        self.slot = LatestSlot()
        self.stop_event = Event()
        self.error = None
        self.thread = Thread(target=self._run, name='v4l2-drain', daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        failures = 0
        try:
            while not self.stop_event.is_set():
                ok, image = self.capture.read()
                if not ok or image is None:
                    failures += 1
                    if failures >= 30:
                        raise RuntimeError('30 consecutive camera read failures; restart camera node')
                    self.stop_event.wait(0.01)
                    continue
                failures = 0
                # Receipt after read/decode, NOT exposure or kernel V4L2 timestamp.
                stamp = self.clock()
                self.slot.put(Frame(image, stamp))
        except Exception as exc:
            self.error = str(exc)
        finally:
            # Release on the reader thread: concurrent release/read can crash OpenCV.
            self.capture.release()

    def close(self, timeout=2.0):
        self.stop_event.set()
        self.thread.join(timeout)
        return not self.thread.is_alive()
