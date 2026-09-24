import asyncio
import time

from urc_can.core import Joint, Reply
from urc_can.worker import FeedbackWorker


JOINTS = tuple(Joint(f'j{i}', i) for i in range(1, 7))


class FakeTransport:
    def __init__(self, behavior='good'):
        self.behavior = behavior
        self.loops = []
        self.closed = False
        self.cancelled = False

    async def query(self):
        self.loops.append(asyncio.get_running_loop())
        if self.behavior == 'timeout':
            try:
                await asyncio.sleep(10)
            finally:
                self.cancelled = True
        if self.behavior == 'error':
            raise OSError('bus disconnected')
        data = [Reply(j.can_id, 0, 0, 0, 0) for j in JOINTS]
        return data[:-1] if self.behavior == 'partial' else data

    def close(self):
        self.closed = True
        self.loops.append(asyncio.get_running_loop())


def test_failure_invalidates_previous_sample_and_recovery():
    async def scenario():
        fake = FakeTransport()
        worker = FeedbackWorker(JOINTS, lambda: fake, timeout=0.01)
        assert await worker.cycle_once(fake)
        last_good = worker.snapshot().last_good_at
        for behavior in ('partial', 'error', 'timeout'):
            fake.behavior = behavior
            assert not await worker.cycle_once(fake)
            state = worker.snapshot()
            assert state.sample is None
            assert state.last_good_at == last_good
            assert state.error
        assert fake.cancelled
        fake.behavior = 'good'
        assert await worker.cycle_once(fake)
        assert worker.snapshot().failures == 3
    asyncio.run(scenario())


def test_real_failure_latches_and_closes_without_retry_or_actuation():
    for behavior in ('partial', 'error', 'timeout'):
        fake = FakeTransport(behavior)
        worker = FeedbackWorker(JOINTS, lambda: fake, timeout=0.01, latch_failures=True)
        asyncio.run(worker.run())
        assert fake.closed
        assert len(fake.loops) == 2  # one query, one close
        assert worker.snapshot().sample is None


def test_persistent_loop_startup_shutdown():
    fake = FakeTransport()
    construction = []

    def factory():
        construction.append(asyncio.get_running_loop())
        return fake

    worker = FeedbackWorker(JOINTS, factory, period=0.005)
    worker.start()
    deadline = time.monotonic() + 2
    while worker.snapshot().sequence < 3 and time.monotonic() < deadline:
        time.sleep(0.005)
    worker.close()
    assert worker.snapshot().sequence >= 3
    assert fake.closed
    assert all(loop is construction[0] for loop in fake.loops)


def test_initialization_failure_is_diagnostic():
    def factory():
        raise RuntimeError('no device')
    worker = FeedbackWorker(JOINTS, factory)
    asyncio.run(worker.run())
    assert 'no device' in worker.snapshot().error
    assert worker.snapshot().diagnostic(time.monotonic(), 0.5)[0] == 3
