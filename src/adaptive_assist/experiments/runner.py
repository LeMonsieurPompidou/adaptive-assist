"""Deterministic open- and closed-loop execution for the scalar joint plant."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

from adaptive_assist.dynamics import JointState, JointTorques, OneDofJointModel
from adaptive_assist.experiments.records import (
    ExperimentMetadata,
    ExperimentResult,
    ExperimentSample,
)
from adaptive_assist.experiments.reference import JointReference
from adaptive_assist.experiments.scenario import ScenarioConfig

if TYPE_CHECKING:
    from adaptive_assist.controllers import JointController

INTEGRATOR_NAME = "semi_implicit_euler"


def run_open_loop_experiment(
    scenario: ScenarioConfig,
    model: OneDofJointModel,
) -> ExperimentResult:
    """Execute configured torques against the plant without controller logic."""
    return _run_experiment(
        scenario,
        model,
        lambda _state, _reference: scenario.torques,
    )


def run_closed_loop_experiment(
    scenario: ScenarioConfig,
    model: OneDofJointModel,
    controller: JointController,
) -> ExperimentResult:
    """Execute controller-requested assistive torque against the joint plant."""

    def controller_torques(
        state: JointState,
        reference: JointReference,
    ) -> JointTorques:
        controller_output = controller.compute(state, reference)
        return JointTorques(
            human_torque_n_m=scenario.torques.human_torque_n_m,
            assistive_torque_n_m=(controller_output.requested_assistive_torque_n_m),
            disturbance_torque_n_m=scenario.torques.disturbance_torque_n_m,
        )

    return _run_experiment(scenario, model, controller_torques)


def _run_experiment(
    scenario: ScenarioConfig,
    model: OneDofJointModel,
    torque_provider: Callable[[JointState, JointReference], JointTorques],
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
        applied_torques = torque_provider(state, reference)
        acceleration_rad_s2 = model.angular_acceleration_rad_s2(
            state,
            applied_torques,
        )
        samples.append(
            ExperimentSample(
                time_s=time_s,
                actual_state=state,
                reference=reference,
                applied_torques=applied_torques,
                angular_acceleration_rad_s2=acceleration_rad_s2,
            )
        )

        if step_index < integration_steps:
            state = model.step(state, applied_torques, scenario.time_step_s)

    metadata = ExperimentMetadata(
        scenario_schema_version=scenario.schema_version,
        integrator_name=INTEGRATOR_NAME,
        reference_type=type(scenario.reference).__name__,
        duration_s=scenario.duration_s,
        time_step_s=scenario.time_step_s,
        integration_steps=integration_steps,
    )
    return ExperimentResult(
        scenario_name=scenario.scenario_name,
        samples=tuple(samples),
        metadata=metadata,
    )
