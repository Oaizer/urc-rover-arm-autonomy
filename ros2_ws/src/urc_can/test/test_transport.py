import asyncio
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from urc_can.core import Joint
from urc_can.transport import MoteusQueryTransport


JOINTS = tuple(Joint(f'j{i}', i) for i in range(1, 7))


class FakeAPI:
    F32, INT8 = 'float32', 'int8'
    Register = SimpleNamespace(POSITION=1, VELOCITY=2, FAULT=3, MODE=4)
    QueryResolution = SimpleNamespace

    def __init__(self):
        self.calls = []
        self.closed = False
        self.results = [SimpleNamespace(id=i, values={1: 0.5, 2: -0.2, 3: 0, 4: 0})
                        for i in range(1, 7)]

    def FdcanusbDevice(self, **kwargs):
        self.calls.append(('fdcanusb', kwargs))
        return SimpleNamespace(close=self.close)

    def PythonCanDevice(self, **kwargs):
        self.calls.append(('socketcan', kwargs))
        return SimpleNamespace(close=self.close)

    def Transport(self, devices):
        self.calls.append(('transport', len(devices)))
        return self

    def Controller(self, *, id, transport, query_resolution):
        assert transport is self
        assert vars(query_resolution) == dict(position=self.F32, velocity=self.F32,
                                              fault=self.INT8, mode=self.INT8)
        # No actuation methods exist: any accidental motion/stop call fails the test.
        return SimpleNamespace(make_query=lambda: ('query', id))

    async def cycle(self, commands):
        self.calls.append(('cycle', commands))
        return self.results

    def close(self):
        self.closed = True
        self.calls.append(('close',))


@pytest.mark.parametrize('backend,channel,options', [
    ('fdcanusb', '/dev/example', {'path': '/dev/example'}),
    ('socketcan', 'can7', {'interface': 'socketcan', 'channel': 'can7',
                         'fd': True, 'ignore_config': True}),
])
def test_only_batched_queries_and_connection_close(backend, channel, options):
    api = FakeAPI()
    transport = MoteusQueryTransport(JOINTS, backend, channel, api=api)
    data = asyncio.run(transport.query())
    transport.close()
    assert api.calls == [(backend, options), ('transport', 1),
                         ('cycle', [('query', i) for i in range(1, 7)]), ('close',)]
    assert [r.can_id for r in data] == list(range(1, 7))
    assert data[0].position_rev == 0.5


@pytest.mark.parametrize('register', [1, 2, 3, 4])
def test_missing_register_never_defaults_to_zero(register):
    api = FakeAPI()
    del api.results[0].values[register]
    transport = MoteusQueryTransport(JOINTS, 'socketcan', 'can0', api=api)
    with pytest.raises(KeyError):
        asyncio.run(transport.query())
    transport.close()
    assert api.closed


@pytest.mark.parametrize('mode', ['motion', 'real', 'simulated', 'trajectory'])
def test_refused_before_device_open(mode):
    api = FakeAPI()
    with pytest.raises(ValueError):
        MoteusQueryTransport(JOINTS, 'socketcan', 'can0', mode=mode, api=api)
    assert api.calls == []


def test_real_moteus_query_wire_is_read_only_if_installed():
    api = pytest.importorskip('moteus')
    # Exercise installed command encoding, replacing the device and bus with fakes.
    from moteus.protocol import parse_registers
    bus = FakeAPI()
    with patch.object(api, 'PythonCanDevice', return_value=object()), \
            patch.object(api, 'Transport', return_value=bus):
        transport = MoteusQueryTransport(JOINTS, 'socketcan', 'unused', api=api)
        for controller in transport.controllers:
            parsed = parse_registers(controller.make_query().data)
            assert not parsed.command
            assert not parsed.response
            requested = {register for register, _ in parsed.query}
            assert {api.Register.POSITION, api.Register.VELOCITY,
                    api.Register.MODE, api.Register.FAULT} <= requested
        transport.close()
