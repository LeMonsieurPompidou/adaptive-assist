"""Shared typed interfaces for joint controllers."""

from dataclasses import dataclass
from typing import Protocol

from adaptive_assist.dynamics import JointState
from adaptive_assist.experiments.reference import JointReference


@dataclass(frozen=True, slots=True)
class ControllerOutput:
    """A controller request preserved for independent command validation.

    The value may be non-finite so an active safety supervisor can record the
    invalid request and replace it with deterministic fallback. The plant-facing
    ``JointTorques`` type continues to reject non-finite applied values.
    """

    requested_assistive_torque_n_m: float


class JointController(Protocol):
    """Interface for deterministic joint-feedback controllers."""

    def compute(
        self,
        state: JointState,
        reference: JointReference,
    ) -> ControllerOutput:
        """Return requested assistive torque for one state and reference."""
        ...
