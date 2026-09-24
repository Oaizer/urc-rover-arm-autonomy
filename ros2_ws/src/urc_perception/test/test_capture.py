from threading import Event

from urc_perception.capture import CaptureWorker, LatestSlot


def test_slot_overwrites_and_consumes_once():
    slot = LatestSlot()
    assert slot.take() is None
    for value in range(1000):
        slot.put(value)
    assert slot.take() == 999
    assert slot.take() is None


def test_capture_drains_without_consumer_and_stamps_after_read():
    done = Event()

    class Capture:
        count = 0
        released = False

        def read(self):
            self.count += 1
            if self.count > 100:
                done.set()
                raise RuntimeError('end of fake stream')
            return True, self.count

        def release(self):
            self.released = True

    capture = Capture()
    worker = CaptureWorker(capture, lambda: capture.count * 10)
    worker.start()
    assert done.wait(2)
    assert worker.close()
    frame = worker.slot.take()
    assert frame.image == 100 and frame.stamp == 1000
    assert capture.released
    assert worker.error == 'end of fake stream'
