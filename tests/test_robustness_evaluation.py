"""Tests for deterministic plant/model mismatch evaluation."""

import math
from dataclasses import replace
from pathlib import Path

import pytest

from adaptive_assist import JointParameters
from adaptive_assist.controllers import (
    ComputedTorqueControllerParameters,
    ImpedanceControllerParameters,
    load_computed_torque_controller_parameters,
    load_impedance_controller_parameters,
)
from adaptive_assist.evaluation import (
    MismatchParameter,
    ParameterVariation,
    RobustnessController,
    RobustnessSweepResult,
    build_robustness_cases,
    load_robustness_sweep_config,
    rmse_degradation_fraction,
    run_robustness_sweep,
    scale_joint_parameters,
    summarize_controller_robustness,
)
from adaptive_assist.experiments import (
    maximum_torque_modification_n_m,
    peak_assistive_torque_n_m,
    peak_requested_assistive_torque_n_m,
    safety_intervention_count,
    safety_intervention_fraction,
    trajectory_tracking_rmse_rad,
)
from adaptive_assist.experiments.scenario import load_scenario
from adaptive_assist.safety import load_safety_limits

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPOSITORY_ROOT / "configs/robustness/model_mismatch_sweep.json"


def _nominal_parameters() -> JointParameters:
    return load_scenario(
        REPOSITORY_ROOT / "configs/scenarios/nominal_tracking.json"
    ).joint_parameters


def _sweep(*, supervised: bool = True) -> RobustnessSweepResult:
    configuration = load_robustness_sweep_config(CONFIG_PATH)
    scenario = load_scenario(REPOSITORY_ROOT / configuration.scenario_config_path)
    safety_limits = (
        load_safety_limits(REPOSITORY_ROOT / configuration.safety_limits_config_path)
        if supervised
        else None
    )
    return run_robustness_sweep(
        configuration,
        scenario,
        load_impedance_controller_parameters(
            REPOSITORY_ROOT / configuration.impedance_controller_config_path
        ),
        load_computed_torque_controller_parameters(
            REPOSITORY_ROOT / configuration.computed_torque_controller_config_path
        ),
        safety_limits,
    )


@pytest.fixture(scope="module")
def supervised_sweep() -> RobustnessSweepResult:
    return _sweep()


def test_scale_one_reproduces_nominal_parameters() -> None:
    nominal = _nominal_parameters()

    actual = scale_joint_parameters(
        nominal,
        (ParameterVariation(MismatchParameter.INERTIA, 1.0),),
    )

    assert actual == nominal


@pytest.mark.parametrize("parameter", tuple(MismatchParameter))
def test_scaling_one_parameter_changes_only_that_parameter(
    parameter: MismatchParameter,
) -> None:
    nominal = _nominal_parameters()

    actual = scale_joint_parameters(
        nominal,
        (ParameterVariation(parameter, 1.2),),
    )

    for candidate in MismatchParameter:
        expected = (
            getattr(nominal, candidate.value) * 1.2
            if candidate is parameter
            else getattr(nominal, candidate.value)
        )
        assert getattr(actual, candidate.value) == pytest.approx(expected)
    assert actual.gravitational_acceleration_m_s2 == (
        nominal.gravitational_acceleration_m_s2
    )
    assert actual.rest_angle_rad == nominal.rest_angle_rad


def test_parameter_scaling_does_not_mutate_nominal_parameters() -> None:
    nominal = _nominal_parameters()
    original = replace(nominal)

    scale_joint_parameters(
        nominal,
        (ParameterVariation(MismatchParameter.MASS, 1.2),),
    )

    assert nominal == original


def test_invalid_derived_parameters_use_joint_parameter_validation() -> None:
    with pytest.raises(ValueError, match="inertia_kg_m2"):
        scale_joint_parameters(
            _nominal_parameters(),
            (ParameterVariation(MismatchParameter.INERTIA, 0.0),),
        )


def test_combined_mismatch_applies_exact_configured_changes() -> None:
    configuration = load_robustness_sweep_config(CONFIG_PATH)
    nominal = _nominal_parameters()

    actual = scale_joint_parameters(
        nominal,
        configuration.combined_mismatch.variations,
    )

    expected_scales = {
        variation.parameter: variation.scale_factor
        for variation in configuration.combined_mismatch.variations
    }
    for parameter in MismatchParameter:
        assert getattr(actual, parameter.value) == pytest.approx(
            getattr(nominal, parameter.value) * expected_scales[parameter]
        )
    assert actual.gravitational_acceleration_m_s2 == (
        nominal.gravitational_acceleration_m_s2
    )
    assert actual.rest_angle_rad == nominal.rest_angle_rad


def test_cases_are_unique_and_nominal_executes_once() -> None:
    cases = build_robustness_cases(load_robustness_sweep_config(CONFIG_PATH))

    assert len(cases) == 12
    assert sum(case.is_nominal for case in cases) == 1
    assert cases[0].case_name == "nominal"
    assert cases[-1].case_name == "combined_moderate_mismatch"


def test_computed_torque_nominal_model_remains_fixed_across_cases(
    supervised_sweep: RobustnessSweepResult,
) -> None:
    nominal = supervised_sweep.nominal_scenario.joint_parameters
    computed_runs = supervised_sweep.runs_for_controller(
        RobustnessController.COMPUTED_TORQUE
    )

    assert all(
        run.nominal_controller_model_parameters == nominal for run in computed_runs
    )
    assert any(run.actual_plant_parameters != nominal for run in computed_runs)
    assert all(
        run.nominal_controller_model_parameters is None
        for run in supervised_sweep.runs_for_controller(RobustnessController.IMPEDANCE)
    )


@pytest.mark.parametrize(
    "parameter",
    [MismatchParameter.INERTIA, MismatchParameter.MASS],
)
def test_actual_parameter_changes_do_not_leak_into_computed_nominal_model(
    supervised_sweep: RobustnessSweepResult,
    parameter: MismatchParameter,
) -> None:
    nominal = supervised_sweep.nominal_scenario.joint_parameters
    mismatch_run = next(
        run
        for run in supervised_sweep.runs_for_controller(
            RobustnessController.COMPUTED_TORQUE
        )
        if run.case.varied_parameter is parameter and run.case.scale_factor == 1.2
    )

    assert getattr(mismatch_run.actual_plant_parameters, parameter.value) != getattr(
        nominal,
        parameter.value,
    )
    assert mismatch_run.nominal_controller_model_parameters == nominal


def test_corresponding_controller_cases_preserve_fair_conditions(
    supervised_sweep: RobustnessSweepResult,
) -> None:
    impedance_runs = {
        run.case.case_name: run
        for run in supervised_sweep.runs_for_controller(RobustnessController.IMPEDANCE)
    }
    computed_runs = {
        run.case.case_name: run
        for run in supervised_sweep.runs_for_controller(
            RobustnessController.COMPUTED_TORQUE
        )
    }

    assert impedance_runs.keys() == computed_runs.keys()
    for case_name, impedance_run in impedance_runs.items():
        computed_run = computed_runs[case_name]
        assert impedance_run.scenario is computed_run.scenario
        assert impedance_run.actual_plant_parameters == (
            computed_run.actual_plant_parameters
        )
        assert (
            impedance_run.scenario.initial_state == computed_run.scenario.initial_state
        )
        assert impedance_run.scenario.reference == computed_run.scenario.reference
        assert impedance_run.scenario.duration_s == computed_run.scenario.duration_s
        assert impedance_run.scenario.time_step_s == computed_run.scenario.time_step_s
        assert impedance_run.scenario.torques == computed_run.scenario.torques
        assert impedance_run.safety_limits is computed_run.safety_limits
        assert tuple(
            sample.reference for sample in impedance_run.experiment_result.samples
        ) == tuple(
            sample.reference for sample in computed_run.experiment_result.samples
        )


def test_non_parameter_conditions_remain_fixed_across_all_cases(
    supervised_sweep: RobustnessSweepResult,
) -> None:
    nominal = supervised_sweep.nominal_scenario

    for run in supervised_sweep.runs:
        assert run.scenario.initial_state is nominal.initial_state
        assert run.scenario.reference is nominal.reference
        assert run.scenario.duration_s == nominal.duration_s
        assert run.scenario.time_step_s == nominal.time_step_s
        assert run.scenario.torques is nominal.torques
        assert run.safety_limits is supervised_sweep.safety_limits
        assert (
            run.actual_plant_parameters.gravitational_acceleration_m_s2
            == nominal.joint_parameters.gravitational_acceleration_m_s2
        )
        assert (
            run.actual_plant_parameters.rest_angle_rad
            == nominal.joint_parameters.rest_angle_rad
        )


def test_feedback_gains_remain_equal_and_fixed(
    supervised_sweep: RobustnessSweepResult,
) -> None:
    impedance_runs = supervised_sweep.runs_for_controller(
        RobustnessController.IMPEDANCE
    )
    computed_runs = supervised_sweep.runs_for_controller(
        RobustnessController.COMPUTED_TORQUE
    )

    assert all(
        run.controller_parameters == impedance_runs[0].controller_parameters
        for run in impedance_runs
    )
    assert all(
        run.controller_parameters == computed_runs[0].controller_parameters
        for run in computed_runs
    )
    impedance_parameters = impedance_runs[0].controller_parameters
    computed_parameters = computed_runs[0].controller_parameters
    assert impedance_parameters.proportional_gain_n_m_per_rad == (
        computed_parameters.proportional_gain_n_m_per_rad
    )
    assert impedance_parameters.derivative_gain_n_m_s_per_rad == (
        computed_parameters.derivative_gain_n_m_s_per_rad
    )


def test_mismatched_feedback_gains_are_rejected() -> None:
    configuration = load_robustness_sweep_config(CONFIG_PATH)
    scenario = load_scenario(REPOSITORY_ROOT / configuration.scenario_config_path)

    with pytest.raises(ValueError, match="matching"):
        run_robustness_sweep(
            configuration,
            scenario,
            ImpedanceControllerParameters(20.0, 4.0),
            ComputedTorqueControllerParameters(21.0, 4.0),
            None,
        )


def test_repeated_sweeps_produce_identical_summaries(
    supervised_sweep: RobustnessSweepResult,
) -> None:
    repeated = _sweep()

    assert repeated == supervised_sweep


def test_case_and_controller_order_is_deterministic(
    supervised_sweep: RobustnessSweepResult,
) -> None:
    case_names = tuple(
        case.case_name
        for case in build_robustness_cases(supervised_sweep.configuration)
    )

    assert (
        tuple(
            run.case.case_name
            for run in supervised_sweep.runs_for_controller(
                RobustnessController.IMPEDANCE
            )
        )
        == case_names
    )
    assert tuple(run.controller_name for run in supervised_sweep.runs) == (
        (RobustnessController.IMPEDANCE,) * len(case_names)
        + (RobustnessController.COMPUTED_TORQUE,) * len(case_names)
    )


def test_robustness_metrics_match_underlying_experiment_results(
    supervised_sweep: RobustnessSweepResult,
) -> None:
    for run in supervised_sweep.runs:
        experiment = run.experiment_result
        assert run.tracking_rmse_rad == trajectory_tracking_rmse_rad(experiment)
        assert run.peak_requested_assistive_torque_n_m == (
            peak_requested_assistive_torque_n_m(experiment)
        )
        assert run.peak_applied_assistive_torque_n_m == (
            peak_assistive_torque_n_m(experiment)
        )
        assert run.intervention_count == safety_intervention_count(experiment)
        assert run.intervention_fraction == safety_intervention_fraction(experiment)
        assert run.maximum_torque_modification_n_m == (
            maximum_torque_modification_n_m(experiment)
        )


def test_degradation_fraction_for_known_values() -> None:
    assert rmse_degradation_fraction(1.2, 1.0) == pytest.approx(0.2)
    assert rmse_degradation_fraction(0.8, 1.0) == pytest.approx(-0.2)


def test_zero_nominal_rmse_is_handled_explicitly() -> None:
    assert rmse_degradation_fraction(0.0, 0.0) == 0.0
    assert math.isinf(rmse_degradation_fraction(0.1, 0.0))


def test_nominal_degradation_is_zero(
    supervised_sweep: RobustnessSweepResult,
) -> None:
    for controller_name in RobustnessController:
        assert (
            supervised_sweep.nominal_run(controller_name).rmse_degradation_fraction
            == 0.0
        )


def test_worst_case_summary_selects_maximum_rmse(
    supervised_sweep: RobustnessSweepResult,
) -> None:
    for controller_name in RobustnessController:
        runs = supervised_sweep.runs_for_controller(controller_name)
        expected = max(runs, key=lambda run: run.tracking_rmse_rad)
        summary = summarize_controller_robustness(
            supervised_sweep,
            controller_name,
        )

        assert summary.nominal_rmse_rad == (
            supervised_sweep.nominal_run(controller_name).tracking_rmse_rad
        )
        assert summary.worst_case_name == expected.case.case_name
        assert summary.worst_case_rmse_rad == expected.tracking_rmse_rad
        assert summary.worst_case_rmse_degradation_fraction == (
            expected.rmse_degradation_fraction
        )


def test_unsupervised_mode_is_explicit_and_uses_no_interventions() -> None:
    result = _sweep(supervised=False)

    assert not result.supervision_active
    assert result.safety_limits is None
    assert all(
        not run.experiment_result.metadata.safety_supervision_active
        for run in result.runs
    )
    assert all(run.intervention_count == 0 for run in result.runs)
    assert all(run.maximum_torque_modification_n_m == 0.0 for run in result.runs)
