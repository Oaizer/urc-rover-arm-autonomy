"""Query-only boundary: no motion, stop, configuration or torque-disable API."""

from typing import Protocol, Sequence

from .core import Reply, validate_joints, validate_mode


class QueryTransport(Protocol):
    async def query(self) -> Sequence[Reply]: ...
    def close(self) -> None: ...


class StationaryTransport:
    """Synthetic feedback fixed at joint offsets, zero velocity; no targets."""

    def __init__(self, joints):
        self.joints = validate_joints(joints)

    async def query(self):
        return [Reply(j.can_id, 0.0, 0.0, 0, 0) for j in self.joints]

    def close(self):
        pass


class MoteusQueryTransport:
    def __init__(self, joints, backend, channel, *, mode='read_only', api=None):
        validate_mode(mode, backend, channel)
        if mode != 'read_only':
            raise ValueError('real transport requires read_only mode')
        self.joints = validate_joints(joints)
        if api is None:
            import moteus as api
        # Require the officially documented TransportDevice API. No auto discovery.
        required = ('Transport', 'FdcanusbDevice', 'PythonCanDevice', 'Controller',
                    'QueryResolution', 'Register', 'F32', 'INT8')
        if any(not hasattr(api, name) for name in required):
            raise RuntimeError('unsupported moteus API: TransportDevice API required')
        self.api = api
        device = (api.FdcanusbDevice(path=channel) if backend == 'fdcanusb' else
                  api.PythonCanDevice(interface='socketcan', channel=channel,
                                      fd=True, ignore_config=True))
        try:
            self.bus = api.Transport([device])
            qr = api.QueryResolution()
            qr.position = api.F32
            qr.velocity = api.F32
            qr.fault = api.INT8
            qr.mode = api.INT8
            self.controllers = [api.Controller(id=j.can_id, transport=self.bus,
                                               query_resolution=qr) for j in self.joints]
        except BaseException:
            device.close()
            raise

    async def query(self):
        results = await self.bus.cycle([c.make_query() for c in self.controllers])
        reg = self.api.Register
        # Indexing is deliberate: absent fields must fail, never become zero.
        return [Reply(r.id, r.values[reg.POSITION], r.values[reg.VELOCITY],
                      r.values[reg.FAULT], r.values[reg.MODE]) for r in results]

    def close(self):
        # Close the host connection only. Do not change the motor's state.
        self.bus.close()
