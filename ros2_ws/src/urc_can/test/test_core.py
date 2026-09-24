import asyncio
from dataclasses import replace
import math

import pytest

from urc_can.core import Joint, Reply, Status, full_sample, validate_joints, validate_mode
from urc_can.transport import StationaryTransport


JOINTS = tuple(Joint(f'j{i}', i) for i in range(1, 7))


def replies():
    return [Reply(i, i / 10, -i / 20, 0, 0) for i in range(1, 7)]


@pytest.mark.parametrize('ratio', [0.5, 1.0, 9.0, 100.0])
@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('offset', [-1.2, 0.0, 0.9])
def test_future_command_units_and_roundtrip(ratio, sign, offset):
    joint = Joint('test', 1, ratio, sign, offset)
    # A quarter joint turn away from offset, half a joint turn per second.
    rev, rev_s = joint.future_command(offset + math.pi / 2, math.pi)
    assert rev == pytest.approx(sign * ratio / 4)
    assert rev_s == pytest.approx(sign * ratio / 2)
    assert joint.feedback(rev, rev_s) == pytest.approx((offset + math.pi / 2, math.pi))
    assert joint.future_command(offset, 0) == (0, 0)


@pytest.mark.parametrize('kwargs', [
    {'can_id': 0}, {'can_id': 127}, {'can_id': True}, {'can_id': 1.5},
    {'name': ''}, {'ratio': 0}, {'ratio': -1}, {'ratio': math.inf},
    {'sign': 0}, {'sign': True}, {'offset_rad': math.nan},
])
def test_invalid_joint(kwargs):
    with pytest.raises(ValueError):
        Joint(**dict({'name': 'j', 'can_id': 1}, **kwargs))


def test_unique_six():
    for joints in (JOINTS[:5], JOINTS + JOINTS[:1], JOINTS[:5] + JOINTS[:1],
                   JOINTS[:5] + (Joint('j1', 6),)):
        with pytest.raises(ValueError):
            validate_joints(joints)


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf, None, True])
def test_invalid_numeric_conversion(value):
    for method in (JOINTS[0].feedback, JOINTS[0].future_command):
        with pytest.raises(ValueError):
            method(value, 0)
        with pytest.raises(ValueError):
            method(0, value)


def test_full_sample_reorders_by_configured_id():
    sample = full_sample(JOINTS, reversed(replies()), 10)
    assert sample.names == tuple(j.name for j in JOINTS)
    assert sample.positions == pytest.approx([math.tau * i / 10 for i in range(1, 7)])


@pytest.mark.parametrize('change', [
    {'fault': 1}, {'fault': None}, {'fault': 0.0}, {'mode': 1}, {'mode': 11},
    {'mode': None}, {'mode': 999}, {'position_rev': math.nan},
    {'velocity_rev_s': math.inf}, {'position_rev': None}, {'can_id': 99},
])
def test_reject_bad_full_sample(change):
    data = replies()
    data[0] = replace(data[0], **change)
    with pytest.raises(ValueError):
        full_sample(JOINTS, data, 10)


def test_no_partial_or_duplicate_samples():
    for data in ([], replies()[:-1], replies() + replies()[:1]):
        with pytest.raises(ValueError):
            full_sample(JOINTS, data, 10)


def test_simulation_is_stationary_at_offsets():
    joints = tuple(replace(j, offset_rad=0.1 * j.can_id) for j in JOINTS)
    mock = StationaryTransport(joints)
    first = asyncio.run(mock.query())
    assert first == asyncio.run(mock.query())
    sample = full_sample(joints, first, 10)
    assert sample.positions == tuple(j.offset_rad for j in joints)
    assert sample.velocities == (0.0,) * 6
    assert not hasattr(mock, 'command')


def test_modes_and_explicit_adapter():
    validate_mode('simulated')
    validate_mode('read_only', 'socketcan', 'can0')
    validate_mode('read_only', 'fdcanusb', '/dev/serial/by-id/example')
    for args in [('motion',), ('real',), ('trajectory',), ('read_only',),
                 ('read_only', 'automatic', 'can0'), ('read_only', 'socketcan', '')]:
        with pytest.raises(ValueError):
            validate_mode(*args)


def test_diagnostics_and_publish_gate():
    sample = full_sample(JOINTS, replies(), 10)
    healthy = Status(1, sample, 10, '', 0)
    assert healthy.diagnostic(10.1, 0.5)[0] == 0
    assert healthy.publishable(10.1, 0.5, 0)
    assert not healthy.publishable(10.1, 0.5, 1)
    assert not healthy.publishable(11, 0.5, 0)
    assert healthy.diagnostic(11, 0.5)[0] == 3
    failed = Status(2, None, 10, 'missing replies', 1)
    assert not failed.publishable(10.1, 0.5, 1)
    assert failed.diagnostic(10.1, 0.5)[0] == 2
    assert failed.diagnostic(11, 0.5)[0] == 3
    assert Status().diagnostic(0, 0.5)[0] == 3
