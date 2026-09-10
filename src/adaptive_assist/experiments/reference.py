"""Deterministic joint-reference representations and generators."""

import math
from dataclasses import dataclass
from typing import Protocol


def _require_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite; received {value!r}")


@dataclass(frozen=True, slots=True)
class JointReference:
    """Desired joint kinematics in radians and seconds."""

    angle_rad: float
    angular_velocity_rad_s: float
    angular_acceleration_rad_s2: float

    def __post_init__(self) -> None:
        """Reject non-finite reference values."""
        _require_finite("angle_rad", self.angle_rad)
        _require_finite("angular_velocity_rad_s", self.angular_velocity_rad_s)
        _require_finite(
            "angular_acceleration_rad_s2",
            self.angular_acceleration_rad_s2,
        )


class ReferenceSignal(Protocol):
    """Interface implemented by deterministic joint-reference generators."""

    def evaluate(self, time_s: float) -> JointReference:
        """Return the joint reference at an experiment time in seconds."""
        ...


@dataclass(frozen=True, slots=True)
class ConstantReference:
    """Reference signal that returns one fixed kinematic reference."""

    value: JointReference

    def evaluate(self, time_s: float) -> JointReference:
        """Return the fixed reference after validating the requested time."""
        _require_finite("time_s", time_s)
        if time_s < 0.0:
            raise ValueError("time_s must be greater than or equal to zero")
        return self.value


@dataclass(frozen=True, slots=True)
class SinusoidalReference:
    """Analytic sinusoidal joint-angle reference."""

    amplitude_rad: float
    frequency_hz: float
    offset_rad: float

    def __post_init__(self) -> None:
        """Validate finite, physically useful signal parameters."""
        _require_finite("amplitude_rad", self.amplitude_rad)
        _require_finite("frequency_hz", self.frequency_hz)
        _require_finite("offset_rad", self.offset_rad)
        if self.amplitude_rad < 0.0:
            raise ValueError("amplitude_rad must be greater than or equal to zero")
        if self.frequency_hz <= 0.0:
            raise ValueError("frequency_hz must be greater than zero")

    def evaluate(self, time_s: float) -> JointReference:
        """Evaluate analytic angle, velocity, and acceleration at ``time_s``."""
        _require_finite("time_s", time_s)
        if time_s < 0.0:
            raise ValueError("time_s must be greater than or equal to zero")

        angular_frequency_rad_s = 2.0 * math.pi * self.frequency_hz
        phase_rad = angular_frequency_rad_s * time_s
        return JointReference(
            angle_rad=self.offset_rad + self.amplitude_rad * math.sin(phase_rad),
            angular_velocity_rad_s=(
                self.amplitude_rad * angular_frequency_rad_s * math.cos(phase_rad)
            ),
            angular_acceleration_rad_s2=(
                -self.amplitude_rad * angular_frequency_rad_s**2 * math.sin(phase_rad)
            ),
        )
