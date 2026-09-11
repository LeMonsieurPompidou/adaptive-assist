"""Deterministic model-mismatch evaluation for existing joint controllers."""

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
from adaptive_assist.experiments import (
    ExperimentResult,
    ScenarioConfig,
    maximum_torque_modification_n_m,
    peak_assistive_torque_n_m,
    peak_requested_assistive_torque_n_m,
    run_closed_loop_experiment,
    safety_intervention_count,
    safety_intervention_fraction,
    trajectory_tracking_rmse_rad,
)
from adaptive_assist.safety import SafetyLimits, SafetySupervisor

ROBUSTNESS_CONFIG_SCHEMA_VERSION = 1


class MismatchParameter(StrEnum):
    """Plant parameters supported by the first mismatch sweep."""

    INERTIA = "inertia_kg_m2"
    MASS = "mass_kg"
    CENTER_OF_MASS_DISTANCE = "center_of_mass_distance_m"
    DAMPING = "damping_n_m_s_per_rad"
    STIFFNESS = "stiffness_n_m_per_rad"


class RobustnessController(StrEnum):
    """Controller baselines included in deterministic robustness evaluation."""

    IMPEDANCE = "impedance"
    COMPUTED_TORQUE = "computed_torque"


@dataclass(frozen=True, slots=True)
class ParameterVariation:
    """One multiplicative change to a nominal plant parameter."""

    parameter: MismatchParameter
    scale_factor: float

    def __post_init__(self) -> None:
        """Require a finite factor and leave physical validation to the plant."""
        if not math.isfinite(self.scale_factor):
            raise ValueError("scale_factor must be finite")


@dataclass(frozen=True, slots=True)
class CombinedMismatch:
    """Named deterministic case containing multiple parameter variations."""

    case_name: str
    variations: tuple[ParameterVariation, ...]

    def __post_init__(self) -> None:
        """Validate case identity and unambiguous parameter ownership."""
        if not self.case_name or not self.case_name.strip():
            raise ValueError("combined mismatch case_name must be non-empty")
        if len(self.variations) < 2:
            raise ValueError("combined mismatch must vary at least two parameters")
        parameters = tuple(variation.parameter for variation in self.variations)
        if len(set(parameters)) != len(parameters):
            raise ValueError("combined mismatch parameters must be unique")


@dataclass(frozen=True, slots=True)
class RobustnessSweepConfig:
    """Versioned paths and deterministic mismatch sweep definition."""

    schema_version: int
    scenario_config_path: str
    impedance_controller_config_path: str
    computed_torque_controller_config_path: str
    safety_limits_config_path: str
    safety_supervision_enabled: bool
    varied_parameters: tuple[MismatchParameter, ...]
    scale_factors: tuple[float, ...]
    combined_mismatch: CombinedMismatch

    def __post_init__(self) -> None:
        """Validate a complete, deterministic, one-at-a-time sweep."""
        if self.schema_version != ROBUSTNESS_CONFIG_SCHEMA_VERSION:
            raise ValueError(
                f"schema_version must be {ROBUSTNESS_CONFIG_SCHEMA_VERSION}; "
                f"received {self.schema_version}"
            )
        path_fields = (
            ("scenario_config_path", self.scenario_config_path),
            (
                "impedance_controller_config_path",
                self.impedance_controller_config_path,
            ),
            (
                "computed_torque_controller_config_path",
                self.computed_torque_controller_config_path,
            ),
            ("safety_limits_config_path", self.safety_limits_config_path),
        )
        for name, value in path_fields:
            if not value or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")

        if not self.varied_parameters:
            raise ValueError("varied_parameters must not be empty")
        if len(set(self.varied_parameters)) != len(self.varied_parameters):
            raise ValueError("varied_parameters must not contain duplicates")
        if not self.scale_factors:
            raise ValueError("scale_factors must not be empty")
        if len(set(self.scale_factors)) != len(self.scale_factors):
            raise ValueError("scale_factors must not contain duplicates")
        for scale_factor in self.scale_factors:
            if not math.isfinite(scale_factor) or scale_factor <= 0.0:
                raise ValueError("scale_factors must be finite and greater than zero")
        if 1.0 not in self.scale_factors:
            raise ValueError("scale_factors must include the nominal factor 1.0")

        varied_parameter_set = set(self.varied_parameters)
        for variation in self.combined_mismatch.variations:
            if variation.scale_factor <= 0.0:
                raise ValueError(
                    "combined mismatch scale factors must be greater than zero"
                )
            if variation.parameter not in varied_parameter_set:
                raise ValueError(
                    "combined mismatch parameters must also appear in varied_parameters"
                )


@dataclass(frozen=True, slots=True)
class RobustnessCase:
    """One unique plant-parameter case to execute."""

    case_name: str
    varied_parameter: MismatchParameter | None
    scale_factor: float | None
    variations: tuple[ParameterVariation, ...]

    @property
    def is_nominal(self) -> bool:
        """Return whether this case uses unchanged nominal parameters."""
        return not self.variations

    @property
    def parameter_label(self) -> str:
        """Return a stable human- and machine-readable case category."""
        if self.is_nominal:
            return "nominal"
        if self.varied_parameter is None:
            return "combined"
        return self.varied_parameter.value


ControllerParameters = (
    ImpedanceControllerParameters | ComputedTorqueControllerParameters
)


@dataclass(frozen=True, slots=True)
class RobustnessRunResult:
    """One experiment plus controller-independent robustness metrics."""

    controller_name: RobustnessController
    case: RobustnessCase
    scenario: ScenarioConfig
    controller_parameters: ControllerParameters
    nominal_controller_model_parameters: JointParameters | None
    safety_limits: SafetyLimits | None
    experiment_result: ExperimentResult
    tracking_rmse_rad: float
    rmse_degradation_fraction: float
    peak_requested_assistive_torque_n_m: float
    peak_applied_assistive_torque_n_m: float
    intervention_count: int
    intervention_fraction: float
    maximum_torque_modification_n_m: float

    @property
    def actual_plant_parameters(self) -> JointParameters:
        """Return the parameters used by the plant for state evolution."""
        return self.scenario.joint_parameters


@dataclass(frozen=True, slots=True)
class RobustnessSweepResult:
    """Deterministically ordered results from one configured sweep."""

    configuration: RobustnessSweepConfig
    nominal_scenario: ScenarioConfig
    safety_limits: SafetyLimits | None
    runs: tuple[RobustnessRunResult, ...]

    @property
    def supervision_active(self) -> bool:
        """Return whether one shared supervisor was used for every run."""
        return self.safety_limits is not None

    def runs_for_controller(
        self,
        controller_name: RobustnessController,
    ) -> tuple[RobustnessRunResult, ...]:
        """Return ordered runs for one controller."""
        return tuple(run for run in self.runs if run.controller_name is controller_name)

    def nominal_run(
        self,
        controller_name: RobustnessController,
    ) -> RobustnessRunResult:
        """Return the unique nominal run for one controller."""
        nominal_runs = tuple(
            run
            for run in self.runs_for_controller(controller_name)
            if run.case.is_nominal
        )
        if len(nominal_runs) != 1:
            raise ValueError(
                f"expected exactly one nominal run for {controller_name.value}"
            )
        return nominal_runs[0]


@dataclass(frozen=True, slots=True)
class ControllerRobustnessSummary:
    """Nominal and worst-RMSE summary for one controller."""

    controller_name: RobustnessController
    nominal_rmse_rad: float
    worst_case_name: str
    worst_case_rmse_rad: float
    worst_case_rmse_degradation_fraction: float


def scale_joint_parameters(
    nominal_parameters: JointParameters,
    variations: tuple[ParameterVariation, ...],
) -> JointParameters:
    """Return validated parameters with explicit multiplicative variations."""
    variation_parameters = tuple(variation.parameter for variation in variations)
    if len(set(variation_parameters)) != len(variation_parameters):
        raise ValueError("parameter variations must be unique")

    changes = {
        variation.parameter.value: (
            getattr(nominal_parameters, variation.parameter.value)
            * variation.scale_factor
        )
        for variation in variations
    }
    return replace(nominal_parameters, **changes)


def build_robustness_cases(
    configuration: RobustnessSweepConfig,
) -> tuple[RobustnessCase, ...]:
    """Build unique deterministic cases while executing nominal only once."""
    cases = [
        RobustnessCase(
            case_name="nominal",
            varied_parameter=None,
            scale_factor=1.0,
            variations=(),
        )
    ]
    for parameter in configuration.varied_parameters:
        for scale_factor in configuration.scale_factors:
            if scale_factor == 1.0:
                continue
            variation = ParameterVariation(parameter, scale_factor)
            cases.append(
                RobustnessCase(
                    case_name=f"{parameter.value}_x{scale_factor:g}",
                    varied_parameter=parameter,
                    scale_factor=scale_factor,
                    variations=(variation,),
                )
            )
    cases.append(
        RobustnessCase(
            case_name=configuration.combined_mismatch.case_name,
            varied_parameter=None,
            scale_factor=None,
            variations=configuration.combined_mismatch.variations,
        )
    )
    return tuple(cases)


def rmse_degradation_fraction(
    mismatch_rmse_rad: float,
    nominal_rmse_rad: float,
) -> float:
    """Return relative RMSE change, with explicit zero-baseline behavior."""
    for name, value in (
        ("mismatch_rmse_rad", mismatch_rmse_rad),
        ("nominal_rmse_rad", nominal_rmse_rad),
    ):
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
    if nominal_rmse_rad == 0.0:
        return 0.0 if mismatch_rmse_rad == 0.0 else math.inf
    return (mismatch_rmse_rad - nominal_rmse_rad) / nominal_rmse_rad


def run_robustness_sweep(
    configuration: RobustnessSweepConfig,
    nominal_scenario: ScenarioConfig,
    impedance_parameters: ImpedanceControllerParameters,
    computed_torque_parameters: ComputedTorqueControllerParameters,
    safety_limits: SafetyLimits | None,
) -> RobustnessSweepResult:
    """Run both fixed-gain controllers over deterministic actual-plant cases."""
    _require_matching_feedback_gains(
        impedance_parameters,
        computed_torque_parameters,
    )
    nominal_parameters = nominal_scenario.joint_parameters
    cases_and_scenarios = tuple(
        (
            case,
            replace(
                nominal_scenario,
                joint_parameters=scale_joint_parameters(
                    nominal_parameters,
                    case.variations,
                ),
            ),
        )
        for case in build_robustness_cases(configuration)
    )
    supervisor = None if safety_limits is None else SafetySupervisor(safety_limits)
    impedance = ImpedanceController(impedance_parameters)
    computed_torque = ComputedTorqueController(
        parameters=computed_torque_parameters,
        nominal_model=OneDofJointModel(nominal_parameters),
    )

    runs: list[RobustnessRunResult] = []
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
    for controller_name, controller, controller_parameters in controller_definitions:
        nominal_rmse_rad: float | None = None
        for case, actual_scenario in cases_and_scenarios:
            experiment_result = run_closed_loop_experiment(
                actual_scenario,
                OneDofJointModel(actual_scenario.joint_parameters),
                controller,
                supervisor,
            )
            tracking_rmse_rad = trajectory_tracking_rmse_rad(experiment_result)
            if case.is_nominal:
                nominal_rmse_rad = tracking_rmse_rad
            if nominal_rmse_rad is None:
                raise RuntimeError("nominal robustness case must execute first")
            nominal_model_parameters = (
                computed_torque.nominal_model.parameters
                if controller_name is RobustnessController.COMPUTED_TORQUE
                else None
            )
            runs.append(
                RobustnessRunResult(
                    controller_name=controller_name,
                    case=case,
                    scenario=actual_scenario,
                    controller_parameters=controller_parameters,
                    nominal_controller_model_parameters=nominal_model_parameters,
                    safety_limits=safety_limits,
                    experiment_result=experiment_result,
                    tracking_rmse_rad=tracking_rmse_rad,
                    rmse_degradation_fraction=rmse_degradation_fraction(
                        tracking_rmse_rad,
                        nominal_rmse_rad,
                    ),
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

    return RobustnessSweepResult(
        configuration=configuration,
        nominal_scenario=nominal_scenario,
        safety_limits=safety_limits,
        runs=tuple(runs),
    )


def summarize_controller_robustness(
    sweep_result: RobustnessSweepResult,
    controller_name: RobustnessController,
) -> ControllerRobustnessSummary:
    """Summarize nominal and worst recorded RMSE for one controller."""
    controller_runs = sweep_result.runs_for_controller(controller_name)
    if not controller_runs:
        raise ValueError(f"no runs recorded for {controller_name.value}")
    nominal_run = sweep_result.nominal_run(controller_name)
    worst_run = max(controller_runs, key=lambda run: run.tracking_rmse_rad)
    return ControllerRobustnessSummary(
        controller_name=controller_name,
        nominal_rmse_rad=nominal_run.tracking_rmse_rad,
        worst_case_name=worst_run.case.case_name,
        worst_case_rmse_rad=worst_run.tracking_rmse_rad,
        worst_case_rmse_degradation_fraction=(worst_run.rmse_degradation_fraction),
    )


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
            "robustness comparison requires matching impedance and "
            "computed-torque feedback gains"
        )
