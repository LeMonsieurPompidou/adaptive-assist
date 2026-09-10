"""Deterministic proportional-derivative impedance control."""

import math
from dataclasses import dataclass

from adaptive_assist.controllers.base import ControllerOutput
from adaptive_assist.dynamics import JointState
from adaptive_assist.experiments.reference import JointReference


@dataclass(frozen=True, slots=True)
class ImpedanceControllerParameters:
    """Validated impedance gains expressed with explicit SI units."""

    proportional_gain_n_m_per_rad: float
    derivative_gain_n_m_s_per_rad: float

    def __post_init__(self) -> None:
        """Require finite, non-negative proportional and derivative gains."""
        gains = (
            ("proportional_gain_n_m_per_rad", self.proportional_gain_n_m_per_rad),
            (
                "derivative_gain_n_m_s_per_rad",
                self.derivative_gain_n_m_s_per_rad,
            ),
        )
        for name, value in gains:
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite; received {value!r}")
            if value < 0.0:
                raise ValueError(f"{name} must be greater than or equal to zero")


@dataclass(frozen=True, slots=True)
class ImpedanceController:
    """Compute requested torque from joint position and velocity errors."""

    parameters: ImpedanceControllerParameters

    def compute(
        self,
        state: JointState,
        reference: JointReference,
    ) -> ControllerOutput:
        """Return the unsaturated proportional-plus-derivative torque request."""
        position_error_rad = reference.angle_rad - state.angle_rad
        velocity_error_rad_s = (
            reference.angular_velocity_rad_s - state.angular_velocity_rad_s
        )
        return ControllerOutput(
            requested_assistive_torque_n_m=(
                self.parameters.proportional_gain_n_m_per_rad * position_error_rad
                + self.parameters.derivative_gain_n_m_s_per_rad * velocity_error_rad_s
            )
        )
