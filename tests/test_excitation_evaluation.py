"""Tests for deterministic excitation and disturbance evaluation."""

import math
from dataclasses import replace
from pathlib import Path

import pytest

from adaptive_assist.controllers import (
    load_computed_torque_controller_parameters,
    load_impedance_controller_parameters,
)
from adaptive_assist.evaluation import (
    DisturbanceProfile,
    ExcitationSweepConfig,
    ExcitationSweepResult,
    RobustnessController,
    build_excitation_cases,
    load_excitation_sweep_config,
    run_excitation_sweep,
    summarize_controller_excitation,
)
from adaptive_assist.experiments import (
    SinusoidalReference,
    maximum_torque_modification_n_m,
    peak_absolute_tracking_error_rad,
    peak_assistive_torque_n_m,
    peak_requested_assistive_torque_n_m,
    safety_intervention_count,
    safety_intervention_fraction,
    trajectory_tracking_rmse_rad,
)
from adaptive_assist.experiments.scenario import load_scenario
from adaptive_assist.safety import load_safety_limits

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPOSITORY_ROOT / "configs/robustness/excitation_disturbance_sweep.json"


def _sweep(*, supervised: bool = True) -> ExcitationSweepResult:
    configuration = load_excitation_sweep_config(CONFIG_PATH)
    scenario = load_scenario(REPOSITORY_ROOT / configuration.scenario_config_path)
    safety_limits = (
        load_safety_limits(REPOSITORY_ROOT / configuration.safety_limits_config_path)
        if supervised
        else None
    )
    return run_excitation_sweep(
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
def supervised_sweep() -> ExcitationSweepResult:
    return _sweep()


def test_case_grid_has_expected_size_and_order() -> None:
    configuration = load_excitation_sweep_config(CONFIG_PATH)
    cases = build_excitation_cases(configuration)

    assert len(cases) == 36
    assert cases[0].case_name == "frequency_0.25_hz__inertia_x0.8__none"
    assert cases[-1].case_name == "frequency_1_hz__inertia_x1.2__negative_pulse"
    assert tuple(case.disturbance_profile for case in cases[:3]) == tuple(
        DisturbanceProfile
    )


def test_sweep_runs_72_controller_experiments(
    supervised_sweep: ExcitationSweepResult,
) -> None:
    assert len(supervised_sweep.runs) == 72
    assert (
        len(supervised_sweep.runs_for_controller(RobustnessController.IMPEDANCE)) == 36
    )
    assert (
        len(supervised_sweep.runs_for_controller(RobustnessController.COMPUTED_TORQUE))
        == 36
    )


def test_generated_references_preserve_amplitude_and_analytic_kinematics(
    supervised_sweep: ExcitationSweepResult,
) -> None:
    configuration = supervised_sweep.configuration
    for run in supervised_sweep.runs_for_controller(RobustnessController.IMPEDANCE):
        reference = run.scenario.reference
        assert isinstance(reference, SinusoidalReference)
        assert reference.amplitude_rad == configuration.reference_amplitude_rad
        assert reference.frequency_hz == run.case.frequency_hz
        quarter_period_s = 0.25 / run.case.frequency_hz
        evaluated = reference.evaluate(quarter_period_s)
        angular_frequency_rad_s = 2.0 * math.pi * run.case.frequency_hz
        assert evaluated.angle_rad == pytest.approx(
            configuration.reference_offset_rad + configuration.reference_amplitude_rad
        )
        assert evaluated.angular_velocity_rad_s == pytest.approx(0.0, abs=1e-12)
        assert evaluated.angular_acceleration_rad_s2 == pytest.approx(
            -configuration.reference_amplitude_rad * angular_frequency_rad_s**2
        )


def test_actual_inertia_changes_while_computed_nominal_model_stays_fixed(
    supervised_sweep: ExcitationSweepResult,
) -> None:
    nominal = supervised_sweep.nominal_scenario.joint_parameters
    computed_runs = supervised_sweep.runs_for_controller(
        RobustnessController.COMPUTED_TORQUE
    )

    assert all(
        run.actual_plant_parameters.inertia_kg_m2
        == pytest.approx(nominal.inertia_kg_m2 * run.case.inertia_scale_factor)
        for run in computed_runs
    )
    assert all(
        run.nominal_controller_model_parameters == nominal for run in computed_runs
    )
    assert all(
        run.nominal_controller_model_parameters is None
        for run in supervised_sweep.runs_for_controller(RobustnessController.IMPEDANCE)
    )


def test_only_actual_inertia_changes(
    supervised_sweep: ExcitationSweepResult,
) -> None:
    nominal = supervised_sweep.nominal_scenario.joint_parameters
    for run in supervised_sweep.runs:
        assert (
            replace(
                run.actual_plant_parameters,
                inertia_kg_m2=nominal.inertia_kg_m2,
            )
            == nominal
        )


def test_corresponding_controllers_have_equivalent_conditions(
    supervised_sweep: ExcitationSweepResult,
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
        assert impedance_run.case is computed_run.case
        assert impedance_run.scenario is computed_run.scenario
        assert impedance_run.safety_limits is computed_run.safety_limits
        assert (
            impedance_run.controller_parameters.proportional_gain_n_m_per_rad
            == computed_run.controller_parameters.proportional_gain_n_m_per_rad
        )
        assert (
            impedance_run.controller_parameters.derivative_gain_n_m_s_per_rad
            == computed_run.controller_parameters.derivative_gain_n_m_s_per_rad
        )


def test_metrics_match_underlying_experiment_results(
    supervised_sweep: ExcitationSweepResult,
) -> None:
    for run in supervised_sweep.runs:
        result = run.experiment_result
        assert run.tracking_rmse_rad == trajectory_tracking_rmse_rad(result)
        assert run.peak_tracking_error_rad == peak_absolute_tracking_error_rad(result)
        assert run.peak_requested_assistive_torque_n_m == (
            peak_requested_assistive_torque_n_m(result)
        )
        assert run.peak_applied_assistive_torque_n_m == (
            peak_assistive_torque_n_m(result)
        )
        assert run.intervention_count == safety_intervention_count(result)
        assert run.intervention_fraction == safety_intervention_fraction(result)
        assert run.maximum_torque_modification_n_m == (
            maximum_torque_modification_n_m(result)
        )


def test_no_disturbance_has_no_recovery_status(
    supervised_sweep: ExcitationSweepResult,
) -> None:
    no_disturbance_runs = tuple(
        run
        for run in supervised_sweep.runs
        if run.case.disturbance_profile is DisturbanceProfile.NONE
    )

    assert all(not run.recovery_applicable for run in no_disturbance_runs)
    assert all(run.recovery_time_s is None for run in no_disturbance_runs)
    assert all(run.recovered is None for run in no_disturbance_runs)


def test_repeated_sweeps_are_identical_and_ordered(
    supervised_sweep: ExcitationSweepResult,
) -> None:
    repeated = _sweep()

    assert repeated == supervised_sweep
    assert tuple(run.controller_name for run in repeated.runs[:36]) == (
        (RobustnessController.IMPEDANCE,) * 36
    )
    assert tuple(run.controller_name for run in repeated.runs[36:]) == (
        (RobustnessController.COMPUTED_TORQUE,) * 36
    )


def test_unsupervised_mode_is_explicit() -> None:
    result = _sweep(supervised=False)

    assert not result.supervision_active
    assert all(run.safety_limits is None for run in result.runs)
    assert all(run.intervention_count == 0 for run in result.runs)
    assert all(run.maximum_torque_modification_n_m == 0.0 for run in result.runs)
    assert all(
        sample.requested_assistive_torque_n_m
        == sample.applied_torques.assistive_torque_n_m
        for run in result.runs
        for sample in run.experiment_result.samples
    )


def test_controller_summary_selects_recorded_extrema(
    supervised_sweep: ExcitationSweepResult,
) -> None:
    summary = summarize_controller_excitation(
        supervised_sweep, RobustnessController.COMPUTED_TORQUE
    )
    runs = supervised_sweep.runs_for_controller(RobustnessController.COMPUTED_TORQUE)

    assert summary.worst_rmse_run.tracking_rmse_rad == max(
        run.tracking_rmse_rad for run in runs
    )
    assert summary.worst_peak_error_run.peak_tracking_error_rad == max(
        run.peak_tracking_error_rad for run in runs
    )
    assert summary.most_interventions_run.intervention_count == max(
        run.intervention_count for run in runs
    )


def test_pulse_must_end_within_scenario() -> None:
    configuration = load_excitation_sweep_config(CONFIG_PATH)
    scenario = load_scenario(REPOSITORY_ROOT / configuration.scenario_config_path)
    invalid_configuration: ExcitationSweepConfig = replace(
        configuration,
        disturbance_start_time_s=scenario.duration_s,
    )

    with pytest.raises(ValueError, match="must end within"):
        run_excitation_sweep(
            invalid_configuration,
            scenario,
            load_impedance_controller_parameters(
                REPOSITORY_ROOT / configuration.impedance_controller_config_path
            ),
            load_computed_torque_controller_parameters(
                REPOSITORY_ROOT / configuration.computed_torque_controller_config_path
            ),
            None,
        )
