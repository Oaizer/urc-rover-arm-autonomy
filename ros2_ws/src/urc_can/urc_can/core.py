"""Pure units, validation and full-sample policy. No ROS or moteus imports."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('expected a finite number')
    if not math.isfinite(value):
        raise ValueError('expected a finite number')
    return value


@dataclass(frozen=True)
class Joint:
    name: str
    can_id: int
    ratio: float = 1.0
    sign: int = 1
    offset_rad: float = 0.0

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('joint name must be nonempty')
        if type(self.can_id) is not int or not 1 <= self.can_id <= 126:
            raise ValueError('CAN IDs must be unicast integers in [1, 126]')
        if finite(self.ratio) <= 0:
            raise ValueError('ratio must be positive')
        if isinstance(self.sign, bool) or self.sign not in (-1, 1):
            raise ValueError('sign must be -1 or +1')
        finite(self.offset_rad)

    def feedback(self, position_rev, velocity_rev_s):
        """ratio = controller POSITION revolutions per joint revolution."""
        position = self.offset_rad + self.sign * math.tau * finite(position_rev) / self.ratio
        velocity = self.sign * math.tau * finite(velocity_rev_s) / self.ratio
        return finite(position), finite(velocity)

    def future_command(self, position_rad, velocity_rad_s):
        """Math only: radians -> controller revolutions; never sends a command."""
        position = self.sign * (finite(position_rad) - self.offset_rad) * self.ratio / math.tau
        velocity = self.sign * finite(velocity_rad_s) * self.ratio / math.tau
        return finite(position), finite(velocity)


def validate_joints(joints):
    joints = tuple(joints)
    if len(joints) != 6:
        raise ValueError('exactly six joints required')
    if len({j.can_id for j in joints}) != 6 or len({j.name for j in joints}) != 6:
        raise ValueError('joint names and CAN IDs must be unique')
    return joints


def validate_mode(mode, backend='', channel=''):
    if mode not in ('simulated', 'read_only'):
        raise ValueError('motion refused: a commissioned controller is required; '
                         'only simulated and read_only are supported')
    if mode == 'read_only' and (backend not in ('fdcanusb', 'socketcan') or not channel.strip()):
        raise ValueError('read_only requires explicit fdcanusb/socketcan backend and channel')


@dataclass(frozen=True)
class Reply:
    can_id: int
    position_rev: float
    velocity_rev_s: float
    fault: int
    mode: int


@dataclass(frozen=True)
class Sample:
    names: Tuple[str, ...]
    positions: Tuple[float, ...]
    velocities: Tuple[float, ...]
    observed_at: float


def full_sample(joints, replies, observed_at):
    """Reject an entire cycle on missing, duplicate, unknown or bad replies."""
    by_id = {}
    expected = {j.can_id for j in joints}
    for reply in replies:
        if type(reply.can_id) is not int or reply.can_id not in expected or reply.can_id in by_id:
            raise ValueError(f'duplicate or unexpected reply ID: {reply.can_id}')
        if type(reply.fault) is not int or reply.fault != 0:
            raise ValueError(f'controller {reply.can_id} fault: {reply.fault}')
        # moteus Mode.FAULT == 1; query both mode and fault, require both valid.
        if type(reply.mode) is not int or reply.mode not in (0, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15):
            raise ValueError(f'controller {reply.can_id} fault/unknown mode: {reply.mode}')
        by_id[reply.can_id] = reply
    if set(by_id) != expected:
        raise ValueError(f'missing replies: {sorted(expected - set(by_id))}')
    converted = [j.feedback(by_id[j.can_id].position_rev, by_id[j.can_id].velocity_rev_s)
                 for j in joints]
    return Sample(tuple(j.name for j in joints), tuple(p for p, _ in converted),
                  tuple(v for _, v in converted), finite(observed_at))


@dataclass(frozen=True)
class Status:
    sequence: int = 0
    sample: Optional[Sample] = None
    last_good_at: Optional[float] = None
    error: str = 'waiting for first full sample'
    failures: int = 0

    def diagnostic(self, now, stale_after):
        if self.last_good_at is None or now - self.last_good_at > stale_after:
            return 3, 'STALE: ' + (self.error or 'feedback expired')
        if self.error:
            return 2, self.error
        return 0, 'full sample valid'

    def publishable(self, now, stale_after, last_sequence):
        return (self.sample is not None and not self.error and
                self.sequence != last_sequence and
                0 <= now - self.sample.observed_at <= stale_after)
