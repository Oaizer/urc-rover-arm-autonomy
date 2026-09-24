"""Rest-to-rest joint trajectory shared by the planner and simulation backend."""
import numpy as np


class Motion:
    def __init__(self, start, end, started, velocity=.35, acceleration=.5, duration=None):
        self.start, self.end = np.asarray(start, float).copy(), np.asarray(end, float).copy()
        if self.start.shape != (6,) or self.end.shape != (6,) or not np.isfinite([*self.start, *self.end, started, velocity, acceleration]).all():
            raise ValueError('finite six-joint endpoints and limits required')
        if velocity <= 0 or acceleration <= 0:
            raise ValueError('positive limits required')
        self.started = started
        distance = float(np.max(np.abs(self.end - self.start)))
        minimum = max(.5, 1.875 * distance / velocity, np.sqrt(5.774 * distance / acceleration))
        if duration is not None and (not np.isfinite(duration) or duration < minimum - 1e-8):
            raise ValueError('trajectory duration violates velocity/acceleration limits')
        self.duration = minimum if duration is None else float(duration)

    def sample(self, now):
        u = float(np.clip((now - self.started) / self.duration, 0, 1))
        s = 10*u**3 - 15*u**4 + 6*u**5
        ds = (30*u**2 - 60*u**3 + 30*u**4) / self.duration
        return self.start + s * (self.end - self.start), ds * (self.end - self.start), u >= 1
