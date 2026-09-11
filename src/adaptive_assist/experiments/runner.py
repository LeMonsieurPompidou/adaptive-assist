"""Deterministic open- and closed-loop execution for the scalar joint plant."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from adaptive_assist.dynamics import JointState, JointTorques, OneDofJointModel
from adaptive_assist.experiments.records import (
    ExperimentMetadata,
    ExperimentResult,
    ExperimentSample,
)
from adaptive_assist.experiments.reference import JointReference
from adaptive_assist.experiments.scenario import ScenarioConfig
from adaptive_assist.safety import SafetyInterventionReason, SafetySupervisor

if TYPE_CHECKING:
    from adaptive_assist.controllers import JointController

INTEGRATOR_NAME = "semi_implicit_euler"


@dataclass(frozen=True, slots=True)
class _ResolvedTorques:
    """Internal requested/applied command resolution for one sample."""

    requested_assistive_torque_n_m: float
    applied_torques: JointTorques
    safety_intervention_reasons: tuple[SafetyInterventionReason, ...] = ()


def run_open_loop_experiment(
    scenario: ScenarioConfig,
    model: OneDofJointModel,
) -> ExperimentResult:
    """Execute configured torques against the plant without controller logic."""
    return _run_experiment(
        scenario,
        model,
        lambda _state, _reference: _ResolvedTorques(
            requested_assistive_torque_n_m=(scenario.torques.assistive_torque_n_m),
            applied_torques=scenario.torques,
        ),
        safety_supervision_active=False,
    )


def run_closed_loop_experiment(
    scenario: ScenarioConfig,
    model: OneDofJointModel,
    controller: JointController,
    safety_supervisor: SafetySupervisor | None = None,
) -> ExperimentResult:
    """Execute a controller with optional independent safety supervision."""

    def controller_torques(
        state: JointState,
        reference: JointReference,
    ) -> _ResolvedTorques:
        controller_output = controller.compute(state, reference)
        requested_torque_n_m = controller_output.requested_assistive_torque_n_m
        safety_result = (
            safety_supervisor.apply(state, requested_torque_n_m)
            if safety_supervisor is not None
            else None
        )
        if safety_result is None and not math.isfinite(requested_torque_n_m):
            raise ValueError(
                "a non-finite controller request requires an active safety supervisor"
            )
        applied_torque_n_m = (
            requested_torque_n_m
            if safety_result is None
            else safety_result.applied_assistive_torque_n_m
        )
        applied_torques = JointTorques(
            human_torque_n_m=scenario.torques.human_torque_n_m,
            assistive_torque_n_m=applied_torque_n_m,
            disturbance_torque_n_m=scenario.torques.disturbance_torque_n_m,
        )
        return _ResolvedTorques(
            requested_assistive_torque_n_m=requested_torque_n_m,
            applied_torques=applied_torques,
            safety_intervention_reasons=(
                () if safety_result is None else safety_result.intervention_reasons
            ),
        )

    return _run_experiment(
        scenario,
        model,
        controller_torques,
        safety_supervision_active=safety_supervisor is not None,
    )


def _run_experiment(
    scenario: ScenarioConfig,
    model: OneDofJointModel,
    torque_provider: Callable[[JointState, JointReference], _ResolvedTorques],
    *,
    safety_supervision_active: bool,
) -> ExperimentResult:
    """Run the shared deterministic fixed-step sampling loop."""
    if model.parameters != scenario.joint_parameters:
        raise ValueError(
            "model parameters must match scenario.joint_parameters for a "
            "reproducible experiment"
        )

    state = scenario.initial_state
    samples: list[ExperimentSample] = []
    integration_steps = scenario.step_count

    for step_index in range(integration_steps + 1):
        time_s = (
            scenario.duration_s
            if step_index == integration_steps
            else step_index * scenario.time_step_s
        )
        reference = scenario.reference.evaluate(time_s)
        resolved_torques = torque_provider(state, reference)
        acceleration_rad_s2 = model.angular_acceleration_rad_s2(
            state,
            resolved_torques.applied_torques,
        )
        samples.append(
            ExperimentSample(
                time_s=time_s,
                actual_state=state,
                reference=reference,
                requested_assistive_torque_n_m=(
                    resolved_torques.requested_assistive_torque_n_m
                ),
                applied_torques=resolved_torques.applied_torques,
                angular_acceleration_rad_s2=acceleration_rad_s2,
                safety_intervention_reasons=(
                    resolved_torques.safety_intervention_reasons
                ),
            )
        )

        if step_index < integration_steps:
            state = model.step(
                state,
                resolved_torques.applied_torques,
                scenario.time_step_s,
            )

    metadata = ExperimentMetadata(
        scenario_schema_version=scenario.schema_version,
        integrator_name=INTEGRATOR_NAME,
        reference_type=type(scenario.reference).__name__,
        duration_s=scenario.duration_s,
        time_step_s=scenario.time_step_s,
        integration_steps=integration_steps,
        safety_supervision_active=safety_supervision_active,
    )
    return ExperimentResult(
        scenario_name=scenario.scenario_name,
        samples=tuple(samples),
        metadata=metadata,
    )
