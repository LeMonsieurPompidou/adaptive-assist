"""Deterministic excitation and disturbance robustness evaluation."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import StrEnum

from adaptive_assist.controllers import (
    ComputedTorqueController,
    ComputedTorqueControllerParameters,
    ImpedanceController,
    ImpedanceControllerParameters,
    JointController,
)
from adaptive_assist.dynamics import JointParameters, OneDofJointModel
from adaptive_assist.evaluation.robustness import (
    MismatchParameter,
    ParameterVariation,
    RobustnessController,
    scale_joint_parameters,
)
from adaptive_assist.experiments import (
    ConstantTorqueSignal,
    ExperimentResult,
    ExperimentSample,
    PulseTorqueSignal,
    ScenarioConfig,
    SinusoidalReference,
    TorqueSignal,
    maximum_torque_modification_n_m,
    peak_absolute_tracking_error_rad,
    peak_assistive_torque_n_m,
    peak_requested_assistive_torque_n_m,
    run_closed_loop_experiment,
    safety_intervention_count,
    safety_intervention_fraction,
    trajectory_tracking_rmse_rad,
)
from adaptive_assist.safety import SafetyLimits, SafetySupervisor

EXCITATION_CONFIG_SCHEMA_VERSION = 1


class DisturbanceProfile(StrEnum):
    """Deterministic disturbance profiles in the excitation study."""

    NONE = "none"
    POSITIVE_PULSE = "positive_pulse"
    NEGATIVE_PULSE = "negative_pulse"


@dataclass(frozen=True, slots=True)
class ExcitationSweepConfig:
    """Versioned inputs for the deterministic excitation study."""

    schema_version: int
    scenario_config_path: str
    impedance_controller_config_path: str
    computed_torque_controller_config_path: str
    safety_limits_config_path: str
    safety_supervision_enabled: bool
    reference_frequencies_hz: tuple[float, ...]
    inertia_scale_factors: tuple[float, ...]
    reference_amplitude_rad: float
    reference_offset_rad: float
    disturbance_profiles: tuple[DisturbanceProfile, ...]
    disturbance_pulse_magnitude_n_m: float
    disturbance_start_time_s: float
    disturbance_duration_s: float
    recovery_tolerance_rad: float
    recovery_dwell_duration_s: float

    def __post_init__(self) -> None:
        """Validate paths, grid values, disturbance timing, and recovery rules."""
        if self.schema_version != EXCITATION_CONFIG_SCHEMA_VERSION:
            raise ValueError(
                f"schema_version must be {EXCITATION_CONFIG_SCHEMA_VERSION}; "
                f"received {self.schema_version}"
            )
        for path_name, path_value in (
            ("scenario_config_path", self.scenario_config_path),
            ("impedance_controller_config_path", self.impedance_controller_config_path),
            (
                "computed_torque_controller_config_path",
                self.computed_torque_controller_config_path,
            ),
            ("safety_limits_config_path", self.safety_limits_config_path),
        ):
            if not path_value or not path_value.strip():
                raise ValueError(f"{path_name} must be a non-empty string")

        _require_positive_unique_values(
            "reference_frequencies_hz", self.reference_frequencies_hz
        )
        _require_positive_unique_values(
            "inertia_scale_factors", self.inertia_scale_factors
        )
        if 1.0 not in self.inertia_scale_factors:
            raise ValueError("inertia_scale_factors must include nominal factor 1.0")
        _require_finite("reference_amplitude_rad", self.reference_amplitude_rad)
        _require_finite("reference_offset_rad", self.reference_offset_rad)
        if self.reference_amplitude_rad <= 0.0:
            raise ValueError("reference_amplitude_rad must be greater than zero")

        if len(set(self.disturbance_profiles)) != len(self.disturbance_profiles):
            raise ValueError("disturbance_profiles must not contain duplicates")
        if set(self.disturbance_profiles) != set(DisturbanceProfile):
            raise ValueError(
                "disturbance_profiles must contain none, positive_pulse, and "
                "negative_pulse"
            )

        for numeric_name, numeric_value in (
            (
                "disturbance_pulse_magnitude_n_m",
                self.disturbance_pulse_magnitude_n_m,
            ),
            ("disturbance_duration_s", self.disturbance_duration_s),
            ("recovery_tolerance_rad", self.recovery_tolerance_rad),
            ("recovery_dwell_duration_s", self.recovery_dwell_duration_s),
        ):
            _require_finite(numeric_name, numeric_value)
            if numeric_value <= 0.0:
                raise ValueError(f"{numeric_name} must be greater than zero")
        _require_finite("disturbance_start_time_s", self.disturbance_start_time_s)
        if self.disturbance_start_time_s < 0.0:
            raise ValueError(
                "disturbance_start_time_s must be greater than or equal to zero"
            )
        if not math.isfinite(self.disturbance_end_time_s):
            raise ValueError("disturbance end time must be finite")

    @property
    def disturbance_end_time_s(self) -> float:
        """Return the exclusive end time shared by pulse profiles."""
        return self.disturbance_start_time_s + self.disturbance_duration_s


@dataclass(frozen=True, slots=True)
class ExcitationCase:
    """One unique frequency, inertia, and disturbance combination."""

    case_name: str
    frequency_hz: float
    inertia_scale_factor: float
    disturbance_profile: DisturbanceProfile
    disturbance_signal: TorqueSignal


ControllerParameters = (
    ImpedanceControllerParameters | ComputedTorqueControllerParameters
)


@dataclass(frozen=True, slots=True)
class ExcitationRunResult:
    """One excitation experiment and its controller-independent metrics."""

    controller_name: RobustnessController
    case: ExcitationCase
    scenario: ScenarioConfig
    controller_parameters: ControllerParameters
    nominal_controller_model_parameters: JointParameters | None
    safety_limits: SafetyLimits | None
    experiment_result: ExperimentResult
    tracking_rmse_rad: float
    peak_tracking_error_rad: float
    recovery_time_s: float | None
    recovery_applicable: bool
    peak_requested_assistive_torque_n_m: float
    peak_applied_assistive_torque_n_m: float
    intervention_count: int
    intervention_fraction: float
    maximum_torque_modification_n_m: float

    @property
    def recovered(self) -> bool | None:
        """Return recovery status, or ``None`` when no pulse was applied."""
        if not self.recovery_applicable:
            return None
        return self.recovery_time_s is not None

    @property
    def actual_plant_parameters(self) -> JointParameters:
        """Return parameters used by the plant for state evolution."""
        return self.scenario.joint_parameters


@dataclass(frozen=True, slots=True)
class ExcitationSweepResult:
    """Deterministically ordered results for one supervision mode."""

    configuration: ExcitationSweepConfig
    nominal_scenario: ScenarioConfig
    safety_limits: SafetyLimits | None
    runs: tuple[ExcitationRunResult, ...]

    @property
    def supervision_active(self) -> bool:
        """Return whether the shared safety supervisor was active."""
        return self.safety_limits is not None

    def runs_for_controller(
        self, controller_name: RobustnessController
    ) -> tuple[ExcitationRunResult, ...]:
        """Return ordered results for one controller."""
        return tuple(run for run in self.runs if run.controller_name is controller_name)


@dataclass(frozen=True, slots=True)
class ControllerExcitationSummary:
    """Deterministic extrema and recovery outcomes for one controller."""

    controller_name: RobustnessController
    worst_rmse_run: ExcitationRunResult
    worst_peak_error_run: ExcitationRunResult
    longest_successful_recovery_run: ExcitationRunResult | None
    failed_recovery_runs: tuple[ExcitationRunResult, ...]
    most_interventions_run: ExcitationRunResult


def build_excitation_cases(
    configuration: ExcitationSweepConfig,
) -> tuple[ExcitationCase, ...]:
    """Build the configured deterministic cross-product in stable order."""
    cases: list[ExcitationCase] = []
    for frequency_hz in configuration.reference_frequencies_hz:
        for inertia_scale_factor in configuration.inertia_scale_factors:
            for disturbance_profile in configuration.disturbance_profiles:
                disturbance_signal: TorqueSignal
                if disturbance_profile is DisturbanceProfile.NONE:
                    disturbance_signal = ConstantTorqueSignal(0.0)
                else:
                    sign = (
                        1.0
                        if disturbance_profile is DisturbanceProfile.POSITIVE_PULSE
                        else -1.0
                    )
                    disturbance_signal = PulseTorqueSignal(
                        pulse_torque_n_m=(
                            sign * configuration.disturbance_pulse_magnitude_n_m
                        ),
                        start_time_s=configuration.disturbance_start_time_s,
                        duration_s=configuration.disturbance_duration_s,
                    )
                cases.append(
                    ExcitationCase(
                        case_name=(
                            f"frequency_{frequency_hz:g}_hz__"
                            f"inertia_x{inertia_scale_factor:g}__"
                            f"{disturbance_profile.value}"
                        ),
                        frequency_hz=frequency_hz,
                        inertia_scale_factor=inertia_scale_factor,
                        disturbance_profile=disturbance_profile,
                        disturbance_signal=disturbance_signal,
                    )
                )
    return tuple(cases)


def disturbance_recovery_time_s(
    result: ExperimentResult,
    *,
    pulse_end_time_s: float,
    tolerance_rad: float,
    dwell_duration_s: float,
) -> float | None:
    """Return sample-defined recovery time relative to pulse end, if achieved.

    Recovery starts at the first recorded sample at or after pulse end whose
    absolute tracking error stays within the inclusive tolerance band through a
    recorded sample at least ``dwell_duration_s`` later.
    """
    for name, value in (
        ("pulse_end_time_s", pulse_end_time_s),
        ("tolerance_rad", tolerance_rad),
        ("dwell_duration_s", dwell_duration_s),
    ):
        _require_finite(name, value)
    if pulse_end_time_s < 0.0:
        raise ValueError("pulse_end_time_s must be greater than or equal to zero")
    if tolerance_rad <= 0.0:
        raise ValueError("tolerance_rad must be greater than zero")
    if dwell_duration_s <= 0.0:
        raise ValueError("dwell_duration_s must be greater than zero")
    if not result.samples:
        raise ValueError("experiment result must contain at least one sample")

    post_pulse_samples = tuple(
        sample
        for sample in result.samples
        if sample.time_s > pulse_end_time_s
        or math.isclose(
            sample.time_s,
            pulse_end_time_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    )
    for candidate_index, candidate in enumerate(post_pulse_samples):
        if _absolute_tracking_error_rad(candidate) > tolerance_rad:
            continue
        dwell_end_time_s = candidate.time_s + dwell_duration_s
        for sample in post_pulse_samples[candidate_index:]:
            if _absolute_tracking_error_rad(sample) > tolerance_rad:
                break
            if sample.time_s > dwell_end_time_s or math.isclose(
                sample.time_s,
                dwell_end_time_s,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                return max(0.0, candidate.time_s - pulse_end_time_s)
    return None


def run_excitation_sweep(
    configuration: ExcitationSweepConfig,
    nominal_scenario: ScenarioConfig,
    impedance_parameters: ImpedanceControllerParameters,
    computed_torque_parameters: ComputedTorqueControllerParameters,
    safety_limits: SafetyLimits | None,
) -> ExcitationSweepResult:
    """Run the fixed-gain controllers over the excitation cross-product."""
    _require_matching_feedback_gains(impedance_parameters, computed_torque_parameters)
    if configuration.disturbance_end_time_s > nominal_scenario.duration_s:
        raise ValueError("disturbance pulse must end within scenario duration")

    nominal_parameters = nominal_scenario.joint_parameters
    cases_and_scenarios = tuple(
        (
            case,
            replace(
                nominal_scenario,
                scenario_name=case.case_name,
                joint_parameters=scale_joint_parameters(
                    nominal_parameters,
                    (
                        ParameterVariation(
                            MismatchParameter.INERTIA,
                            case.inertia_scale_factor,
                        ),
                    ),
                ),
                reference=SinusoidalReference(
                    amplitude_rad=configuration.reference_amplitude_rad,
                    frequency_hz=case.frequency_hz,
                    offset_rad=configuration.reference_offset_rad,
                ),
                torques=replace(
                    nominal_scenario.torques,
                    disturbance_torque_n_m=0.0,
                ),
            ),
        )
        for case in build_excitation_cases(configuration)
    )
    supervisor = None if safety_limits is None else SafetySupervisor(safety_limits)
    impedance = ImpedanceController(impedance_parameters)
    computed_torque = ComputedTorqueController(
        parameters=computed_torque_parameters,
        nominal_model=OneDofJointModel(nominal_parameters),
    )
    controller_definitions: tuple[
        tuple[RobustnessController, JointController, ControllerParameters], ...
    ] = (
        (RobustnessController.IMPEDANCE, impedance, impedance_parameters),
        (
            RobustnessController.COMPUTED_TORQUE,
            computed_torque,
            computed_torque_parameters,
        ),
    )

    runs: list[ExcitationRunResult] = []
    for controller_name, controller, controller_parameters in controller_definitions:
        for case, actual_scenario in cases_and_scenarios:
            experiment_result = run_closed_loop_experiment(
                actual_scenario,
                OneDofJointModel(actual_scenario.joint_parameters),
                controller,
                supervisor,
                disturbance_signal=case.disturbance_signal,
            )
            recovery_applicable = (
                case.disturbance_profile is not DisturbanceProfile.NONE
            )
            recovery_time_s = (
                disturbance_recovery_time_s(
                    experiment_result,
                    pulse_end_time_s=configuration.disturbance_end_time_s,
                    tolerance_rad=configuration.recovery_tolerance_rad,
                    dwell_duration_s=configuration.recovery_dwell_duration_s,
                )
                if recovery_applicable
                else None
            )
            runs.append(
                ExcitationRunResult(
                    controller_name=controller_name,
                    case=case,
                    scenario=actual_scenario,
                    controller_parameters=controller_parameters,
                    nominal_controller_model_parameters=(
                        computed_torque.nominal_model.parameters
                        if controller_name is RobustnessController.COMPUTED_TORQUE
                        else None
                    ),
                    safety_limits=safety_limits,
                    experiment_result=experiment_result,
                    tracking_rmse_rad=trajectory_tracking_rmse_rad(experiment_result),
                    peak_tracking_error_rad=peak_absolute_tracking_error_rad(
                        experiment_result
                    ),
                    recovery_time_s=recovery_time_s,
                    recovery_applicable=recovery_applicable,
                    peak_requested_assistive_torque_n_m=(
                        peak_requested_assistive_torque_n_m(experiment_result)
                    ),
                    peak_applied_assistive_torque_n_m=(
                        peak_assistive_torque_n_m(experiment_result)
                    ),
                    intervention_count=safety_intervention_count(experiment_result),
                    intervention_fraction=safety_intervention_fraction(
                        experiment_result
                    ),
                    maximum_torque_modification_n_m=(
                        maximum_torque_modification_n_m(experiment_result)
                    ),
                )
            )

    return ExcitationSweepResult(
        configuration=configuration,
        nominal_scenario=nominal_scenario,
        safety_limits=safety_limits,
        runs=tuple(runs),
    )


def summarize_controller_excitation(
    sweep_result: ExcitationSweepResult,
    controller_name: RobustnessController,
) -> ControllerExcitationSummary:
    """Return deterministic extrema and pulse-recovery outcomes."""
    controller_runs = sweep_result.runs_for_controller(controller_name)
    if not controller_runs:
        raise ValueError(f"no runs recorded for {controller_name.value}")
    successful_recoveries = tuple(
        run
        for run in controller_runs
        if run.recovery_applicable and run.recovery_time_s is not None
    )
    return ControllerExcitationSummary(
        controller_name=controller_name,
        worst_rmse_run=max(controller_runs, key=lambda run: run.tracking_rmse_rad),
        worst_peak_error_run=max(
            controller_runs, key=lambda run: run.peak_tracking_error_rad
        ),
        longest_successful_recovery_run=(
            max(successful_recoveries, key=lambda run: run.recovery_time_s or 0.0)
            if successful_recoveries
            else None
        ),
        failed_recovery_runs=tuple(
            run
            for run in controller_runs
            if run.recovery_applicable and run.recovery_time_s is None
        ),
        most_interventions_run=max(
            controller_runs, key=lambda run: run.intervention_count
        ),
    )


def _absolute_tracking_error_rad(sample: ExperimentSample) -> float:
    return abs(sample.actual_state.angle_rad - sample.reference.angle_rad)


def _require_matching_feedback_gains(
    impedance_parameters: ImpedanceControllerParameters,
    computed_torque_parameters: ComputedTorqueControllerParameters,
) -> None:
    if (
        impedance_parameters.proportional_gain_n_m_per_rad
        != computed_torque_parameters.proportional_gain_n_m_per_rad
        or impedance_parameters.derivative_gain_n_m_s_per_rad
        != computed_torque_parameters.derivative_gain_n_m_s_per_rad
    ):
        raise ValueError(
            "excitation comparison requires matching impedance and "
            "computed-torque feedback gains"
        )


def _require_positive_unique_values(name: str, values: tuple[float, ...]) -> None:
    if not values:
        raise ValueError(f"{name} must not be empty")
    if len(set(values)) != len(values):
        raise ValueError(f"{name} must not contain duplicates")
    for value in values:
        _require_finite(name, value)
        if value <= 0.0:
            raise ValueError(f"{name} values must be greater than zero")


def _require_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
