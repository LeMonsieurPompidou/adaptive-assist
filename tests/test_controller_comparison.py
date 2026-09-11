"""Tests for equivalent conditions in the controller comparison."""

from pathlib import Path

from adaptive_assist import OneDofJointModel
from adaptive_assist.controllers import (
    ImpedanceController,
    load_impedance_controller_parameters,
)
from adaptive_assist.experiments import (
    load_scenario,
    run_closed_loop_experiment,
    run_open_loop_experiment,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def test_comparison_changes_only_assistive_torque_generation() -> None:
    scenario = load_scenario(
        REPOSITORY_ROOT / "configs/scenarios/nominal_tracking.json"
    )
    parameters = load_impedance_controller_parameters(
        REPOSITORY_ROOT / "configs/controllers/impedance_baseline.json"
    )
    open_loop_result = run_open_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
    )
    impedance_result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        ImpedanceController(parameters),
    )

    assert open_loop_result.scenario_name == impedance_result.scenario_name
    assert open_loop_result.metadata == impedance_result.metadata
    assert len(open_loop_result.samples) == len(impedance_result.samples)
    assert open_loop_result.samples[0].actual_state == (
        impedance_result.samples[0].actual_state
    )
    assert tuple(sample.time_s for sample in open_loop_result.samples) == tuple(
        sample.time_s for sample in impedance_result.samples
    )
    assert tuple(sample.reference for sample in open_loop_result.samples) == tuple(
        sample.reference for sample in impedance_result.samples
    )
    assert all(
        sample.applied_torques.assistive_torque_n_m
        == scenario.torques.assistive_torque_n_m
        == 0.0
        for sample in open_loop_result.samples
    )
    assert any(
        sample.applied_torques.assistive_torque_n_m != 0.0
        for sample in impedance_result.samples
    )
    assert all(
        open_sample.applied_torques.human_torque_n_m
        == impedance_sample.applied_torques.human_torque_n_m
        and open_sample.applied_torques.disturbance_torque_n_m
        == impedance_sample.applied_torques.disturbance_torque_n_m
        for open_sample, impedance_sample in zip(
            open_loop_result.samples,
            impedance_result.samples,
            strict=True,
        )
    )
