"""Deterministic scalar torque signals for experiment inputs."""

import math
from dataclasses import dataclass
from typing import Protocol


def _require_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite; received {value!r}")


class TorqueSignal(Protocol):
    """Interface implemented by deterministic time-varying torque inputs."""

    def evaluate(self, time_s: float) -> float:
        """Return torque in newton metres at an experiment time."""
        ...


@dataclass(frozen=True, slots=True)
class ConstantTorqueSignal:
    """Return one constant finite torque at every non-negative time."""

    torque_n_m: float

    def __post_init__(self) -> None:
        """Validate the configured torque."""
        _require_finite("torque_n_m", self.torque_n_m)

    def evaluate(self, time_s: float) -> float:
        """Return the configured torque after validating experiment time."""
        _require_valid_time(time_s)
        return self.torque_n_m


@dataclass(frozen=True, slots=True)
class PulseTorqueSignal:
    """Apply finite torque over the half-open interval ``[start, end)``."""

    pulse_torque_n_m: float
    start_time_s: float
    duration_s: float

    def __post_init__(self) -> None:
        """Validate finite pulse magnitude and positive timing."""
        _require_finite("pulse_torque_n_m", self.pulse_torque_n_m)
        _require_finite("start_time_s", self.start_time_s)
        _require_finite("duration_s", self.duration_s)
        if self.start_time_s < 0.0:
            raise ValueError("start_time_s must be greater than or equal to zero")
        if self.duration_s <= 0.0:
            raise ValueError("duration_s must be greater than zero")
        if not math.isfinite(self.end_time_s):
            raise ValueError("pulse end time must be finite")

    @property
    def end_time_s(self) -> float:
        """Return the exclusive pulse-end time in seconds."""
        return self.start_time_s + self.duration_s

    def evaluate(self, time_s: float) -> float:
        """Return pulse torque inside ``[start, end)`` and zero elsewhere."""
        _require_valid_time(time_s)
        at_or_after_start = time_s > self.start_time_s or math.isclose(
            time_s,
            self.start_time_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        before_end = time_s < self.end_time_s and not math.isclose(
            time_s,
            self.end_time_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        if at_or_after_start and before_end:
            return self.pulse_torque_n_m
        return 0.0


def _require_valid_time(time_s: float) -> None:
    _require_finite("time_s", time_s)
    if time_s < 0.0:
        raise ValueError("time_s must be greater than or equal to zero")
