"""Shared typed interfaces for joint controllers."""

import math
from dataclasses import dataclass
from typing import Protocol

from adaptive_assist.dynamics import JointState
from adaptive_assist.experiments.reference import JointReference


@dataclass(frozen=True, slots=True)
class ControllerOutput:
    """A controller-requested assistive torque before future supervision."""

    requested_assistive_torque_n_m: float

    def __post_init__(self) -> None:
        """Reject non-finite controller requests."""
        if not math.isfinite(self.requested_assistive_torque_n_m):
            raise ValueError("requested_assistive_torque_n_m must be finite")


class JointController(Protocol):
    """Interface for deterministic joint-feedback controllers."""

    def compute(
        self,
        state: JointState,
        reference: JointReference,
    ) -> ControllerOutput:
        """Return requested assistive torque for one state and reference."""
        ...
