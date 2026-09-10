"""Immutable records produced by deterministic experiments."""

import math
from dataclasses import dataclass

from adaptive_assist.dynamics import JointState, JointTorques
from adaptive_assist.experiments.reference import JointReference


@dataclass(frozen=True, slots=True)
class ExperimentSample:
    """One timestamped observation from an open-loop experiment."""

    time_s: float
    actual_state: JointState
    reference: JointReference
    applied_torques: JointTorques
    angular_acceleration_rad_s2: float

    def __post_init__(self) -> None:
        """Validate sample time and calculated acceleration."""
        if not math.isfinite(self.time_s) or self.time_s < 0.0:
            raise ValueError("time_s must be finite and greater than or equal to zero")
        if not math.isfinite(self.angular_acceleration_rad_s2):
            raise ValueError("angular_acceleration_rad_s2 must be finite")


@dataclass(frozen=True, slots=True)
class ExperimentMetadata:
    """Deterministic metadata needed to interpret an experiment result."""

    scenario_schema_version: int
    integrator_name: str
    reference_type: str
    duration_s: float
    time_step_s: float
    integration_steps: int


@dataclass(frozen=True, slots=True)
class ExperimentResult:
    """Immutable samples and metadata from one experiment execution."""

    scenario_name: str
    samples: tuple[ExperimentSample, ...]
    metadata: ExperimentMetadata
