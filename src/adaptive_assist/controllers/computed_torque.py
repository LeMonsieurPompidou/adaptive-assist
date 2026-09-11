"""Deterministic model-based computed-torque control."""

import math
from dataclasses import dataclass

from adaptive_assist.controllers.base import ControllerOutput
from adaptive_assist.dynamics import JointState, OneDofJointModel
from adaptive_assist.experiments.reference import JointReference


@dataclass(frozen=True, slots=True)
class ComputedTorqueControllerParameters:
    """Validated computed-torque feedback gains in explicit SI units."""

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
class ComputedTorqueController:
    """Combine nominal-model compensation with tracking-error feedback."""

    parameters: ComputedTorqueControllerParameters
    nominal_model: OneDofJointModel

    def compute(
        self,
        state: JointState,
        reference: JointReference,
    ) -> ControllerOutput:
        """Return an unsaturated model-based assistive torque request."""
        position_error_rad = reference.angle_rad - state.angle_rad
        velocity_error_rad_s = (
            reference.angular_velocity_rad_s - state.angular_velocity_rad_s
        )
        feedback_torque_n_m = (
            self.parameters.proportional_gain_n_m_per_rad * position_error_rad
            + self.parameters.derivative_gain_n_m_s_per_rad * velocity_error_rad_s
        )
        feedforward_inertial_torque_n_m = (
            self.nominal_model.parameters.inertia_kg_m2
            * reference.angular_acceleration_rad_s2
        )
        gravity_compensation_n_m = self.nominal_model.gravity_torque_n_m(
            state.angle_rad
        )
        passive_compensation_n_m = self.nominal_model.passive_torque_n_m(state)

        return ControllerOutput(
            requested_assistive_torque_n_m=math.fsum(
                (
                    feedforward_inertial_torque_n_m,
                    gravity_compensation_n_m,
                    passive_compensation_n_m,
                    feedback_torque_n_m,
                )
            )
        )
