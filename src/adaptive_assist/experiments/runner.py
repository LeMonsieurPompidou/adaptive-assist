"""Deterministic open-loop execution for the scalar joint plant."""

from adaptive_assist.dynamics import OneDofJointModel
from adaptive_assist.experiments.records import (
    ExperimentMetadata,
    ExperimentResult,
    ExperimentSample,
)
from adaptive_assist.experiments.scenario import ScenarioConfig

INTEGRATOR_NAME = "semi_implicit_euler"


def run_open_loop_experiment(
    scenario: ScenarioConfig,
    model: OneDofJointModel,
) -> ExperimentResult:
    """Execute configured torques against the plant without controller logic."""
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
        acceleration_rad_s2 = model.angular_acceleration_rad_s2(
            state,
            scenario.torques,
        )
        samples.append(
            ExperimentSample(
                time_s=time_s,
                actual_state=state,
                reference=reference,
                applied_torques=scenario.torques,
                angular_acceleration_rad_s2=acceleration_rad_s2,
            )
        )

        if step_index < integration_steps:
            state = model.step(state, scenario.torques, scenario.time_step_s)

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
