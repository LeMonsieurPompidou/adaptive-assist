"""Typed scenario configuration and strict standard-library JSON loading."""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import cast

from adaptive_assist.dynamics import JointParameters, JointState, JointTorques
from adaptive_assist.experiments.reference import (
    ConstantReference,
    JointReference,
    ReferenceSignal,
    SinusoidalReference,
)

SCENARIO_SCHEMA_VERSION = 1


class ScenarioConfigError(ValueError):
    """Raised when a scenario file is malformed or unsupported."""


@dataclass(frozen=True, slots=True)
class ScenarioConfig:
    """Deterministic plant, reference, timing, and external-torque configuration."""

    schema_version: int
    scenario_name: str
    duration_s: float
    time_step_s: float
    initial_state: JointState
    joint_parameters: JointParameters
    reference: ReferenceSignal
    torques: JointTorques

    def __post_init__(self) -> None:
        """Validate scenario identity, timing, and fixed-step compatibility."""
        if self.schema_version != SCENARIO_SCHEMA_VERSION:
            raise ValueError(
                f"schema_version must be {SCENARIO_SCHEMA_VERSION}; "
                f"received {self.schema_version}"
            )
        if not self.scenario_name or not self.scenario_name.strip():
            raise ValueError("scenario_name must be a non-empty string")
        if not math.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("duration_s must be finite and greater than zero")
        if not math.isfinite(self.time_step_s) or self.time_step_s <= 0.0:
            raise ValueError("time_step_s must be finite and greater than zero")

        step_ratio = self.duration_s / self.time_step_s
        step_count = round(step_ratio)
        if step_count < 1 or not math.isclose(
            step_ratio,
            float(step_count),
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(
                "duration_s must be an integer multiple of time_step_s for "
                "deterministic fixed-step execution"
            )

    @property
    def step_count(self) -> int:
        """Return the validated number of integration steps."""
        return round(self.duration_s / self.time_step_s)


def load_scenario(path: str | Path) -> ScenarioConfig:
    """Load and strictly validate a versioned JSON scenario file."""
    scenario_path = Path(path)
    try:
        with scenario_path.open(encoding="utf-8") as scenario_file:
            raw_config: object = json.load(scenario_file)
    except json.JSONDecodeError as error:
        raise ScenarioConfigError(
            f"Invalid JSON in scenario file {scenario_path}: {error.msg}"
        ) from error
    except OSError as error:
        raise ScenarioConfigError(
            f"Could not read scenario file {scenario_path}: {error}"
        ) from error

    try:
        return _parse_scenario(raw_config)
    except ScenarioConfigError:
        raise
    except ValueError as error:
        raise ScenarioConfigError(
            f"Invalid scenario configuration in {scenario_path}: {error}"
        ) from error


def _parse_scenario(raw_config: object) -> ScenarioConfig:
    config = _object(raw_config, "scenario")
    _require_exact_keys(
        config,
        {
            "schema_version",
            "scenario_name",
            "duration_s",
            "time_step_s",
            "initial_state",
            "joint_parameters",
            "reference",
            "torques",
        },
        "scenario",
    )

    initial_state_data = _object(config["initial_state"], "initial_state")
    _require_exact_keys(
        initial_state_data,
        {"angle_rad", "angular_velocity_rad_s"},
        "initial_state",
    )

    parameter_data = _object(config["joint_parameters"], "joint_parameters")
    _require_exact_keys(
        parameter_data,
        {
            "inertia_kg_m2",
            "mass_kg",
            "center_of_mass_distance_m",
            "gravitational_acceleration_m_s2",
            "damping_n_m_s_per_rad",
            "stiffness_n_m_per_rad",
            "rest_angle_rad",
        },
        "joint_parameters",
    )

    torque_data = _object(config["torques"], "torques")
    _require_exact_keys(
        torque_data,
        {
            "human_torque_n_m",
            "assistive_torque_n_m",
            "disturbance_torque_n_m",
        },
        "torques",
    )

    return ScenarioConfig(
        schema_version=_integer(config["schema_version"], "schema_version"),
        scenario_name=_string(config["scenario_name"], "scenario_name"),
        duration_s=_number(config["duration_s"], "duration_s"),
        time_step_s=_number(config["time_step_s"], "time_step_s"),
        initial_state=JointState(
            angle_rad=_number(
                initial_state_data["angle_rad"], "initial_state.angle_rad"
            ),
            angular_velocity_rad_s=_number(
                initial_state_data["angular_velocity_rad_s"],
                "initial_state.angular_velocity_rad_s",
            ),
        ),
        joint_parameters=JointParameters(
            inertia_kg_m2=_number(
                parameter_data["inertia_kg_m2"],
                "joint_parameters.inertia_kg_m2",
            ),
            mass_kg=_number(parameter_data["mass_kg"], "joint_parameters.mass_kg"),
            center_of_mass_distance_m=_number(
                parameter_data["center_of_mass_distance_m"],
                "joint_parameters.center_of_mass_distance_m",
            ),
            gravitational_acceleration_m_s2=_number(
                parameter_data["gravitational_acceleration_m_s2"],
                "joint_parameters.gravitational_acceleration_m_s2",
            ),
            damping_n_m_s_per_rad=_number(
                parameter_data["damping_n_m_s_per_rad"],
                "joint_parameters.damping_n_m_s_per_rad",
            ),
            stiffness_n_m_per_rad=_number(
                parameter_data["stiffness_n_m_per_rad"],
                "joint_parameters.stiffness_n_m_per_rad",
            ),
            rest_angle_rad=_number(
                parameter_data["rest_angle_rad"],
                "joint_parameters.rest_angle_rad",
            ),
        ),
        reference=_parse_reference(config["reference"]),
        torques=JointTorques(
            human_torque_n_m=_number(
                torque_data["human_torque_n_m"],
                "torques.human_torque_n_m",
            ),
            assistive_torque_n_m=_number(
                torque_data["assistive_torque_n_m"],
                "torques.assistive_torque_n_m",
            ),
            disturbance_torque_n_m=_number(
                torque_data["disturbance_torque_n_m"],
                "torques.disturbance_torque_n_m",
            ),
        ),
    )


def _parse_reference(raw_reference: object) -> ReferenceSignal:
    reference_data = _object(raw_reference, "reference")
    reference_type = _string(reference_data.get("type"), "reference.type")

    if reference_type == "constant":
        _require_exact_keys(
            reference_data,
            {
                "type",
                "angle_rad",
                "angular_velocity_rad_s",
                "angular_acceleration_rad_s2",
            },
            "reference",
        )
        return ConstantReference(
            JointReference(
                angle_rad=_number(reference_data["angle_rad"], "reference.angle_rad"),
                angular_velocity_rad_s=_number(
                    reference_data["angular_velocity_rad_s"],
                    "reference.angular_velocity_rad_s",
                ),
                angular_acceleration_rad_s2=_number(
                    reference_data["angular_acceleration_rad_s2"],
                    "reference.angular_acceleration_rad_s2",
                ),
            )
        )

    if reference_type == "sinusoidal":
        _require_exact_keys(
            reference_data,
            {"type", "amplitude_rad", "frequency_hz", "offset_rad"},
            "reference",
        )
        return SinusoidalReference(
            amplitude_rad=_number(
                reference_data["amplitude_rad"],
                "reference.amplitude_rad",
            ),
            frequency_hz=_number(
                reference_data["frequency_hz"],
                "reference.frequency_hz",
            ),
            offset_rad=_number(reference_data["offset_rad"], "reference.offset_rad"),
        )

    raise ScenarioConfigError(
        f"reference.type must be 'constant' or 'sinusoidal'; "
        f"received {reference_type!r}"
    )


def _object(value: object, context: str) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ScenarioConfigError(f"{context} must be a JSON object")
    return cast(dict[str, object], value)


def _require_exact_keys(
    value: dict[str, object],
    expected_keys: set[str],
    context: str,
) -> None:
    actual_keys = set(value)
    missing_keys = sorted(expected_keys - actual_keys)
    extra_keys = sorted(actual_keys - expected_keys)
    if missing_keys:
        raise ScenarioConfigError(
            f"{context} is missing required field(s): {', '.join(missing_keys)}"
        )
    if extra_keys:
        raise ScenarioConfigError(
            f"{context} contains unsupported field(s): {', '.join(extra_keys)}"
        )


def _number(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioConfigError(f"{context} must be a JSON number")
    return float(value)


def _integer(value: object, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ScenarioConfigError(f"{context} must be a JSON integer")
    return value


def _string(value: object, context: str) -> str:
    if not isinstance(value, str):
        raise ScenarioConfigError(f"{context} must be a JSON string")
    return value
