"""Integration tests for computed torque through the generic runner."""

from dataclasses import replace
from pathlib import Path

from adaptive_assist import OneDofJointModel
from adaptive_assist.controllers import (
    ComputedTorqueController,
    ImpedanceController,
    JointController,
    load_computed_torque_controller_parameters,
    load_impedance_controller_parameters,
)
from adaptive_assist.experiments import load_scenario, run_closed_loop_experiment

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_PATH = REPOSITORY_ROOT / "configs/scenarios/nominal_tracking.json"


def test_generic_runner_accepts_both_controller_implementations() -> None:
    scenario = load_scenario(SCENARIO_PATH)
    impedance: JointController = ImpedanceController(
        load_impedance_controller_parameters(
            REPOSITORY_ROOT / "configs/controllers/impedance_baseline.json"
        )
    )
    computed_torque: JointController = ComputedTorqueController(
        parameters=load_computed_torque_controller_parameters(
            REPOSITORY_ROOT / "configs/controllers/computed_torque_baseline.json"
        ),
        nominal_model=OneDofJointModel(scenario.joint_parameters),
    )

    impedance_result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        impedance,
    )
    computed_torque_result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        computed_torque,
    )

    assert len(impedance_result.samples) == len(computed_torque_result.samples)
    assert impedance_result.samples[0].time_s == 0.0
    assert computed_torque_result.samples[-1].time_s == scenario.duration_s


def test_controller_nominal_model_can_differ_from_actual_plant() -> None:
    scenario = load_scenario(SCENARIO_PATH)
    actual_plant = OneDofJointModel(scenario.joint_parameters)
    mismatched_nominal_parameters = replace(
        scenario.joint_parameters,
        inertia_kg_m2=scenario.joint_parameters.inertia_kg_m2 * 1.25,
    )
    nominal_controller_model = OneDofJointModel(mismatched_nominal_parameters)
    controller = ComputedTorqueController(
        parameters=load_computed_torque_controller_parameters(
            REPOSITORY_ROOT / "configs/controllers/computed_torque_baseline.json"
        ),
        nominal_model=nominal_controller_model,
    )

    result = run_closed_loop_experiment(scenario, actual_plant, controller)

    assert controller.nominal_model is nominal_controller_model
    assert controller.nominal_model is not actual_plant
    assert controller.nominal_model.parameters != actual_plant.parameters
    assert len(result.samples) == scenario.step_count + 1
