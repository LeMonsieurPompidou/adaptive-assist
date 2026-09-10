"""Tests for deterministic open-loop experiment execution."""

from dataclasses import replace
from itertools import pairwise

import pytest

from adaptive_assist import (
    JointParameters,
    JointState,
    JointTorques,
    OneDofJointModel,
)
from adaptive_assist.experiments import (
    SCENARIO_SCHEMA_VERSION,
    ConstantReference,
    JointReference,
    ScenarioConfig,
    run_open_loop_experiment,
)


def _scenario() -> ScenarioConfig:
    return ScenarioConfig(
        schema_version=SCENARIO_SCHEMA_VERSION,
        scenario_name="runner_test",
        duration_s=0.2,
        time_step_s=0.1,
        initial_state=JointState(0.0, 0.0),
        joint_parameters=JointParameters(
            inertia_kg_m2=2.0,
            mass_kg=0.0,
            center_of_mass_distance_m=0.0,
            gravitational_acceleration_m_s2=9.81,
            damping_n_m_s_per_rad=0.0,
            stiffness_n_m_per_rad=0.0,
            rest_angle_rad=0.0,
        ),
        reference=ConstantReference(JointReference(0.0, 0.0, 0.0)),
        torques=JointTorques(
            human_torque_n_m=0.5,
            assistive_torque_n_m=2.0,
            disturbance_torque_n_m=-0.5,
        ),
    )


def test_runner_records_expected_times_and_sample_count() -> None:
    scenario = _scenario()
    result = run_open_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
    )

    assert len(result.samples) == scenario.step_count + 1
    assert result.samples[0].time_s == 0.0
    assert tuple(sample.time_s for sample in result.samples) == pytest.approx(
        (0.0, 0.1, 0.2)
    )
    assert all(
        later.time_s > earlier.time_s for earlier, later in pairwise(result.samples)
    )


def test_final_sample_uses_configured_duration_exactly() -> None:
    scenario = replace(_scenario(), duration_s=0.3)

    result = run_open_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
    )

    assert result.samples[-1].time_s == scenario.duration_s


def test_runner_is_deterministic_and_records_configured_torques() -> None:
    scenario = _scenario()
    model = OneDofJointModel(scenario.joint_parameters)

    first_result = run_open_loop_experiment(scenario, model)
    second_result = run_open_loop_experiment(scenario, model)

    assert first_result == second_result
    assert all(
        sample.applied_torques == scenario.torques for sample in first_result.samples
    )
    assert first_result.metadata.scenario_schema_version == SCENARIO_SCHEMA_VERSION
    assert first_result.metadata.integrator_name == "semi_implicit_euler"
    assert first_result.metadata.reference_type == "ConstantReference"
    assert first_result.metadata.integration_steps == scenario.step_count


def test_runner_uses_existing_plant_step_behavior() -> None:
    scenario = _scenario()
    result = run_open_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
    )

    assert result.samples[0].angular_acceleration_rad_s2 == pytest.approx(1.0)
    assert result.samples[1].actual_state.angle_rad == pytest.approx(0.01)
    assert result.samples[1].actual_state.angular_velocity_rad_s == pytest.approx(0.1)
    assert result.samples[2].actual_state.angle_rad == pytest.approx(0.03)
    assert result.samples[2].actual_state.angular_velocity_rad_s == pytest.approx(0.2)


def test_runner_does_not_mutate_scenario_inputs() -> None:
    scenario = _scenario()
    initial_state = scenario.initial_state
    torques = scenario.torques

    run_open_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
    )

    assert scenario.initial_state is initial_state
    assert scenario.initial_state == JointState(0.0, 0.0)
    assert scenario.torques is torques


def test_runner_rejects_model_parameter_mismatch() -> None:
    scenario = _scenario()
    different_parameters = JointParameters(
        inertia_kg_m2=3.0,
        mass_kg=0.0,
        center_of_mass_distance_m=0.0,
        gravitational_acceleration_m_s2=9.81,
        damping_n_m_s_per_rad=0.0,
        stiffness_n_m_per_rad=0.0,
        rest_angle_rad=0.0,
    )

    with pytest.raises(ValueError, match="must match"):
        run_open_loop_experiment(scenario, OneDofJointModel(different_parameters))
