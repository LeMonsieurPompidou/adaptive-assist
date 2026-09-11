"""Immutable records produced by deterministic experiments."""

import math
from dataclasses import dataclass

from adaptive_assist.dynamics import JointState, JointTorques
from adaptive_assist.experiments.reference import JointReference
from adaptive_assist.safety import SafetyInterventionReason


@dataclass(frozen=True, slots=True)
class ExperimentSample:
    """One timestamped observation from an experiment."""

    time_s: float
    actual_state: JointState
    reference: JointReference
    requested_assistive_torque_n_m: float
    applied_torques: JointTorques
    angular_acceleration_rad_s2: float
    safety_intervention_reasons: tuple[SafetyInterventionReason, ...] = ()

    def __post_init__(self) -> None:
        """Validate sample timing, command resolution, and acceleration."""
        if not math.isfinite(self.time_s) or self.time_s < 0.0:
            raise ValueError("time_s must be finite and greater than or equal to zero")
        if not math.isfinite(self.angular_acceleration_rad_s2):
            raise ValueError("angular_acceleration_rad_s2 must be finite")
        requested_is_finite = math.isfinite(self.requested_assistive_torque_n_m)
        invalid_reason = SafetyInterventionReason.INVALID_REQUESTED_COMMAND
        if not requested_is_finite and invalid_reason not in (
            self.safety_intervention_reasons
        ):
            raise ValueError(
                "a non-finite requested command requires invalid-command intervention"
            )
        if requested_is_finite and invalid_reason in self.safety_intervention_reasons:
            raise ValueError(
                "invalid-command intervention requires a non-finite requested command"
            )
        if (
            not self.safety_intervention_reasons
            and self.requested_assistive_torque_n_m
            != self.applied_torques.assistive_torque_n_m
        ):
            raise ValueError(
                "a modified assistive torque requires an intervention reason"
            )

    @property
    def safety_intervened(self) -> bool:
        """Return whether supervision modified or replaced this command."""
        return bool(self.safety_intervention_reasons)


@dataclass(frozen=True, slots=True)
class ExperimentMetadata:
    """Deterministic metadata needed to interpret an experiment result."""

    scenario_schema_version: int
    integrator_name: str
    reference_type: str
    duration_s: float
    time_step_s: float
    integration_steps: int
    safety_supervision_active: bool = False


@dataclass(frozen=True, slots=True)
class ExperimentResult:
    """Immutable samples and metadata from one experiment execution."""

    scenario_name: str
    samples: tuple[ExperimentSample, ...]
    metadata: ExperimentMetadata
