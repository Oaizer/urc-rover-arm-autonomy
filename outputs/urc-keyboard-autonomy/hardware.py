"""Hardware boundaries. Imports of moteus are optional until live mode."""

from __future__ import annotations

import asyncio
import time
from typing import Iterable

import numpy as np

from interfaces import JointFeedback, MotionCommand


def controller_to_joint(raw_rad: np.ndarray, output_ratios: np.ndarray,
                        zero_offsets_rad: np.ndarray,
                        direction_signs: np.ndarray) -> np.ndarray:
    """Convert controller position to calibrated output-joint radians."""
    return direction_signs * (raw_rad - zero_offsets_rad) / output_ratios


def joint_to_controller(joint_rad: np.ndarray, output_ratios: np.ndarray,
                        zero_offsets_rad: np.ndarray,
                        direction_signs: np.ndarray) -> np.ndarray:
    """Convert desired output-joint radians to controller radians."""
    return zero_offsets_rad + direction_signs * joint_rad * output_ratios


class SimulatedJoints:
    def __init__(self, count: int = 6) -> None:
        self.position = np.zeros(count, dtype=float)
        self.command = self.position.copy()
        self.last = time.monotonic_ns()
        self.sequence = 0

    def feedback(self) -> JointFeedback:
        now = time.monotonic_ns()
        dt = max((now - self.last) / 1e9, 1e-6)
        velocity = (self.position - self.command) / dt
        self.position = self.command.copy()
        self.last = now
        return JointFeedback(now, self.position.copy(), velocity, ())

    def send(self, command: MotionCommand) -> None:
        self.sequence = command.sequence
        self.command = command.positions_rad.copy()

    def stop(self) -> None:
        self.command = self.position.copy()


class MoteusCanBridge:
    """Small synchronous facade over moteus' async Python client.

    The transport and controller IDs are configuration-driven. The exact
    CubeMars motor and reduction must be selected before this class is enabled.
    """

    def __init__(self, controller_ids: Iterable[int], serial_port: str,
                 output_ratios: Iterable[float] | None = None,
                 zero_offsets_rad: Iterable[float] | None = None,
                 direction_signs: Iterable[float] | None = None,
                 limits_rad: Iterable[Iterable[float]] | None = None) -> None:
        self.ids = tuple(int(x) for x in controller_ids)
        if len(self.ids) != 6:
            raise ValueError("exactly six moteus controller IDs are required")
        self.output_ratios = np.asarray(
            list(output_ratios) if output_ratios is not None else np.ones(6), dtype=float)
        self.zero_offsets = np.asarray(
            list(zero_offsets_rad) if zero_offsets_rad is not None else np.zeros(6), dtype=float)
        self.direction_signs = np.asarray(
            list(direction_signs) if direction_signs is not None else np.ones(6), dtype=float)
        self.limits = np.asarray(
            list(limits_rad) if limits_rad is not None else [[-np.inf, np.inf]] * 6,
            dtype=float)
        if (self.output_ratios.shape != (6,) or self.zero_offsets.shape != (6,)
                or self.direction_signs.shape != (6,) or self.limits.shape != (6, 2)
                or not np.all(np.isfinite(self.output_ratios))
                or not np.all(np.isfinite(self.zero_offsets))
                or not np.all(np.isin(self.direction_signs, (-1.0, 1.0)))
                or np.any(self.output_ratios <= 0.0)
                or np.any(self.limits[:, 0] >= self.limits[:, 1])):
            raise ValueError("ratios, offsets, signs, and joint limits must contain calibrated values")
        try:
            import moteus
        except ImportError as exc:
            raise RuntimeError("install moteus before using live CAN mode") from exc
        self._moteus = moteus
        self._transport = moteus.Fdcanusb(serial_port=serial_port)
        self._controllers = [moteus.Controller(id=i, transport=self._transport) for i in self.ids]
        self._last = JointFeedback(time.monotonic_ns(), np.zeros(6), np.zeros(6), ("not_read",))

    def _run(self, awaitable):
        return asyncio.run(awaitable)

    def feedback(self) -> JointFeedback:
        async def query():
            results = []
            for controller in self._controllers:
                results.append(await controller.set_position(query=True))
            return results

        try:
            results = self._run(query())
            raw_positions = np.array([float(r.values.get(self._moteus.Register.POSITION, 0.0))
                                      for r in results]) * 2.0 * np.pi
            raw_velocities = np.array([float(r.values.get(self._moteus.Register.VELOCITY, 0.0))
                                       for r in results]) * 2.0 * np.pi
            positions = controller_to_joint(raw_positions, self.output_ratios,
                                            self.zero_offsets, self.direction_signs)
            velocities = self.direction_signs * raw_velocities / self.output_ratios
            faults = tuple(str(r.values.get(self._moteus.Register.FAULT, ""))
                           for r in results if r.values.get(self._moteus.Register.FAULT, 0))
            self._last = JointFeedback(time.monotonic_ns(), positions, velocities, faults)
        except Exception as exc:
            self._last = JointFeedback(self._last.timestamp_ns, self._last.positions_rad,
                                       self._last.velocities_rad_s, (f"CAN: {exc}",))
        return self._last

    def send(self, command: MotionCommand) -> None:
        if (command.positions_rad.shape != (6,)
                or np.any(command.positions_rad < self.limits[:, 0])
                or np.any(command.positions_rad > self.limits[:, 1])):
            raise ValueError("motion command violates configured joint limits")
        controller_rad = joint_to_controller(
            command.positions_rad, self.output_ratios,
            self.zero_offsets, self.direction_signs)
        revolutions = controller_rad / (2.0 * np.pi)

        async def send_all():
            for index, (controller, position) in enumerate(zip(self._controllers, revolutions)):
                await controller.set_position(position=float(position),
                                              velocity_limit=command.velocity_limit_rad_s * self.output_ratios[index] / (2*np.pi),
                                              accel_limit=command.acceleration_limit_rad_s2 * self.output_ratios[index] / (2*np.pi),
                                              maximum_torque=command.torque_limit)

        self._run(send_all())

    def stop(self) -> None:
        async def stop_all():
            for controller in self._controllers:
                await controller.set_stop()
        self._run(stop_all())
