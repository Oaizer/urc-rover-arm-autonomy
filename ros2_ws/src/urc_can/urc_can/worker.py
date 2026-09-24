"""One persistent asyncio loop; serial batched cycles and a bounded mailbox."""

import asyncio
import threading
import time

from .core import Status, finite, full_sample, validate_joints


class FeedbackWorker:
    def __init__(self, joints, factory, *, period=0.05, timeout=0.1,
                 latch_failures=False, clock=time.monotonic):
        self.joints = validate_joints(joints)
        if finite(period) <= 0 or finite(timeout) <= 0:
            raise ValueError('period and timeout must be positive')
        self.factory, self.period, self.timeout = factory, period, timeout
        self.latch_failures, self.clock = latch_failures, clock
        self._lock = threading.Lock()
        self._status = Status()
        self._stop = threading.Event()
        self._thread = None

    def snapshot(self):
        with self._lock:
            return self._status

    def _failure(self, exc):
        with self._lock:
            old = self._status
            self._status = Status(old.sequence + 1, None, old.last_good_at,
                                  f'{type(exc).__name__}: {exc}', old.failures + 1)

    async def cycle_once(self, transport):
        try:
            started = self.clock()
            replies = await asyncio.wait_for(transport.query(), timeout=self.timeout)
            ended = self.clock()
            # Also reject a coroutine that blocks or swallows cancellation.
            if ended - started > self.timeout:
                raise TimeoutError('cycle exceeded deadline')
            sample = full_sample(self.joints, replies, ended)
        except Exception as exc:
            self._failure(exc)
            return False
        with self._lock:
            old = self._status
            self._status = Status(old.sequence + 1, sample, sample.observed_at, '', old.failures)
        return True

    async def run(self):
        transport = None
        try:
            # Open, query, and close on this same running event loop/thread.
            transport = self.factory()
            while not self._stop.is_set():
                start = self.clock()
                good = await self.cycle_once(transport)
                if not good and self.latch_failures:
                    break  # Real failures require process restart; no queued reply reuse.
                delay = max(0, self.period - (self.clock() - start))
                # Short sleeps make shutdown responsive without concurrent CAN cycles.
                end = self.clock() + delay
                while not self._stop.is_set() and self.clock() < end:
                    await asyncio.sleep(min(0.02, end - self.clock()))
        except Exception as exc:
            self._failure(exc)
        finally:
            if transport is not None:
                try:
                    transport.close()
                except Exception as exc:
                    self._failure(exc)

    def start(self):
        if self._thread is not None:
            raise RuntimeError('worker already started')
        self._thread = threading.Thread(target=lambda: asyncio.run(self.run()),
                                        name='urc-can-query', daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self.timeout + 1.0)
            if self._thread.is_alive():
                raise RuntimeError('transport did not terminate; worker is unresponsive')
