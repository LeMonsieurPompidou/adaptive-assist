"""Independent deterministic command constraints for simulation experiments."""

import math
from dataclasses import dataclass
from enum import StrEnum

from adaptive_assist.dynamics import JointState


class SafetyInterventionReason(StrEnum):
    """Reasons that the supervisor replaced or modified a command."""

    INVALID_REQUESTED_COMMAND = "invalid_requested_command"
    JOINT_POSITION_LIMIT = "joint_position_limit"
    JOINT_VELOCITY_LIMIT = "joint_velocity_limit"
    TORQUE_LIMIT = "torque_limit"


@dataclass(frozen=True, slots=True)
class SafetyLimits:
    """Validated deterministic limits for the mathematical simulation."""

    max_abs_assistive_torque_n_m: float
    min_joint_angle_rad: float
    max_joint_angle_rad: float
    max_abs_joint_velocity_rad_s: float
    fallback_assistive_torque_n_m: float

    def __post_init__(self) -> None:
        """Require finite and internally consistent simulation limits."""
        values = (
            (
                "max_abs_assistive_torque_n_m",
                self.max_abs_assistive_torque_n_m,
            ),
            ("min_joint_angle_rad", self.min_joint_angle_rad),
            ("max_joint_angle_rad", self.max_joint_angle_rad),
            (
                "max_abs_joint_velocity_rad_s",
                self.max_abs_joint_velocity_rad_s,
            ),
            (
                "fallback_assistive_torque_n_m",
                self.fallback_assistive_torque_n_m,
            ),
        )
        for name, value in values:
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite; received {value!r}")

        if self.max_abs_assistive_torque_n_m < 0.0:
            raise ValueError(
                "max_abs_assistive_torque_n_m must be greater than or equal to zero"
            )
        if self.max_abs_joint_velocity_rad_s < 0.0:
            raise ValueError(
                "max_abs_joint_velocity_rad_s must be greater than or equal to zero"
            )
        if self.min_joint_angle_rad >= self.max_joint_angle_rad:
            raise ValueError(
                "min_joint_angle_rad must be less than max_joint_angle_rad"
            )
        if abs(self.fallback_assistive_torque_n_m) > self.max_abs_assistive_torque_n_m:
            raise ValueError(
                "fallback_assistive_torque_n_m must be within the assistive torque "
                "limit"
            )


@dataclass(frozen=True, slots=True)
class SafetyResult:
    """Resolved assistive torque and deterministic intervention information."""

    requested_assistive_torque_n_m: float
    applied_assistive_torque_n_m: float
    intervention_reasons: tuple[SafetyInterventionReason, ...] = ()

    def __post_init__(self) -> None:
        """Reject incoherent or non-finite resolved command data."""
        requested_is_finite = math.isfinite(self.requested_assistive_torque_n_m)
        if not math.isfinite(self.applied_assistive_torque_n_m):
            raise ValueError("applied_assistive_torque_n_m must be finite")
        if len(set(self.intervention_reasons)) != len(self.intervention_reasons):
            raise ValueError("intervention_reasons must not contain duplicates")
        invalid_reason = SafetyInterventionReason.INVALID_REQUESTED_COMMAND
        if not requested_is_finite and invalid_reason not in self.intervention_reasons:
            raise ValueError(
                "a non-finite requested command requires invalid-command intervention"
            )
        if requested_is_finite and invalid_reason in self.intervention_reasons:
            raise ValueError(
                "invalid-command intervention requires a non-finite requested command"
            )
        if (
            not self.intervention_reasons
            and self.requested_assistive_torque_n_m != self.applied_assistive_torque_n_m
        ):
            raise ValueError("a modified command requires an intervention reason")

    @property
    def intervened(self) -> bool:
        """Return whether the supervisor replaced or modified the request."""
        return bool(self.intervention_reasons)


@dataclass(frozen=True, slots=True)
class SafetySupervisor:
    """Apply deterministic simulation limits independently of controllers."""

    limits: SafetyLimits

    def apply(
        self,
        state: JointState,
        requested_assistive_torque_n_m: float,
    ) -> SafetyResult:
        """Resolve one requested torque using documented precedence rules."""
        if not math.isfinite(requested_assistive_torque_n_m):
            return SafetyResult(
                requested_assistive_torque_n_m=requested_assistive_torque_n_m,
                applied_assistive_torque_n_m=(
                    self.limits.fallback_assistive_torque_n_m
                ),
                intervention_reasons=(
                    SafetyInterventionReason.INVALID_REQUESTED_COMMAND,
                ),
            )

        state_reasons: list[SafetyInterventionReason] = []
        if (
            state.angle_rad < self.limits.min_joint_angle_rad
            or state.angle_rad > self.limits.max_joint_angle_rad
        ):
            state_reasons.append(SafetyInterventionReason.JOINT_POSITION_LIMIT)
        if abs(state.angular_velocity_rad_s) > self.limits.max_abs_joint_velocity_rad_s:
            state_reasons.append(SafetyInterventionReason.JOINT_VELOCITY_LIMIT)
        if state_reasons:
            return SafetyResult(
                requested_assistive_torque_n_m=requested_assistive_torque_n_m,
                applied_assistive_torque_n_m=(
                    self.limits.fallback_assistive_torque_n_m
                ),
                intervention_reasons=tuple(state_reasons),
            )

        torque_limit_n_m = self.limits.max_abs_assistive_torque_n_m
        if abs(requested_assistive_torque_n_m) > torque_limit_n_m:
            applied_torque_n_m = max(
                -torque_limit_n_m,
                min(torque_limit_n_m, requested_assistive_torque_n_m),
            )
            return SafetyResult(
                requested_assistive_torque_n_m=requested_assistive_torque_n_m,
                applied_assistive_torque_n_m=applied_torque_n_m,
                intervention_reasons=(SafetyInterventionReason.TORQUE_LIMIT,),
            )

        return SafetyResult(
            requested_assistive_torque_n_m=requested_assistive_torque_n_m,
            applied_assistive_torque_n_m=requested_assistive_torque_n_m,
        )
