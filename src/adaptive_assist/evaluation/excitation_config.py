"""Strict JSON loading for deterministic excitation robustness sweeps."""

import json
import math
from pathlib import Path
from typing import cast

from adaptive_assist.evaluation.excitation import (
    DisturbanceProfile,
    ExcitationSweepConfig,
)


class ExcitationConfigError(ValueError):
    """Raised when an excitation robustness configuration is invalid."""


def load_excitation_sweep_config(path: str | Path) -> ExcitationSweepConfig:
    """Load and strictly validate a versioned excitation sweep."""
    config_path = Path(path)
    try:
        with config_path.open(encoding="utf-8") as config_file:
            raw_config: object = json.load(config_file)
    except json.JSONDecodeError as error:
        raise ExcitationConfigError(
            f"Invalid JSON in excitation configuration {config_path}: {error.msg}"
        ) from error
    except OSError as error:
        raise ExcitationConfigError(
            f"Could not read excitation configuration {config_path}: {error}"
        ) from error

    try:
        return _parse_configuration(raw_config)
    except ExcitationConfigError:
        raise
    except ValueError as error:
        raise ExcitationConfigError(
            f"Invalid excitation configuration in {config_path}: {error}"
        ) from error


def _parse_configuration(raw_config: object) -> ExcitationSweepConfig:
    config = _object(raw_config, "excitation configuration")
    _require_exact_keys(
        config,
        {
            "schema_version",
            "scenario_config_path",
            "impedance_controller_config_path",
            "computed_torque_controller_config_path",
            "safety_limits_config_path",
            "safety_supervision_enabled",
            "reference_frequencies_hz",
            "inertia_scale_factors",
            "reference_amplitude_rad",
            "reference_offset_rad",
            "disturbance_profiles",
            "disturbance_pulse_magnitude_n_m",
            "disturbance_start_time_s",
            "disturbance_duration_s",
            "recovery_tolerance_rad",
            "recovery_dwell_duration_s",
        },
        "excitation configuration",
    )
    frequencies = _array(config["reference_frequencies_hz"], "reference_frequencies_hz")
    inertia_factors = _array(config["inertia_scale_factors"], "inertia_scale_factors")
    profiles = _array(config["disturbance_profiles"], "disturbance_profiles")

    return ExcitationSweepConfig(
        schema_version=_integer(config["schema_version"], "schema_version"),
        scenario_config_path=_string(
            config["scenario_config_path"], "scenario_config_path"
        ),
        impedance_controller_config_path=_string(
            config["impedance_controller_config_path"],
            "impedance_controller_config_path",
        ),
        computed_torque_controller_config_path=_string(
            config["computed_torque_controller_config_path"],
            "computed_torque_controller_config_path",
        ),
        safety_limits_config_path=_string(
            config["safety_limits_config_path"], "safety_limits_config_path"
        ),
        safety_supervision_enabled=_boolean(
            config["safety_supervision_enabled"], "safety_supervision_enabled"
        ),
        reference_frequencies_hz=tuple(
            _number(value, f"reference_frequencies_hz[{index}]")
            for index, value in enumerate(frequencies)
        ),
        inertia_scale_factors=tuple(
            _number(value, f"inertia_scale_factors[{index}]")
            for index, value in enumerate(inertia_factors)
        ),
        reference_amplitude_rad=_number(
            config["reference_amplitude_rad"], "reference_amplitude_rad"
        ),
        reference_offset_rad=_number(
            config["reference_offset_rad"], "reference_offset_rad"
        ),
        disturbance_profiles=tuple(
            _disturbance_profile(value, f"disturbance_profiles[{index}]")
            for index, value in enumerate(profiles)
        ),
        disturbance_pulse_magnitude_n_m=_number(
            config["disturbance_pulse_magnitude_n_m"],
            "disturbance_pulse_magnitude_n_m",
        ),
        disturbance_start_time_s=_number(
            config["disturbance_start_time_s"], "disturbance_start_time_s"
        ),
        disturbance_duration_s=_number(
            config["disturbance_duration_s"], "disturbance_duration_s"
        ),
        recovery_tolerance_rad=_number(
            config["recovery_tolerance_rad"], "recovery_tolerance_rad"
        ),
        recovery_dwell_duration_s=_number(
            config["recovery_dwell_duration_s"], "recovery_dwell_duration_s"
        ),
    )


def _object(value: object, context: str) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ExcitationConfigError(f"{context} must be a JSON object")
    return cast(dict[str, object], value)


def _array(value: object, context: str) -> list[object]:
    if not isinstance(value, list):
        raise ExcitationConfigError(f"{context} must be a JSON array")
    return cast(list[object], value)


def _require_exact_keys(
    value: dict[str, object], expected_keys: set[str], context: str
) -> None:
    missing_keys = sorted(expected_keys - set(value))
    extra_keys = sorted(set(value) - expected_keys)
    if missing_keys:
        raise ExcitationConfigError(
            f"{context} is missing required field(s): {', '.join(missing_keys)}"
        )
    if extra_keys:
        raise ExcitationConfigError(
            f"{context} contains unsupported field(s): {', '.join(extra_keys)}"
        )


def _disturbance_profile(value: object, context: str) -> DisturbanceProfile:
    profile_name = _string(value, context)
    try:
        return DisturbanceProfile(profile_name)
    except ValueError as error:
        supported = ", ".join(profile.value for profile in DisturbanceProfile)
        raise ExcitationConfigError(
            f"{context} must be one of: {supported}; received {profile_name!r}"
        ) from error


def _number(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ExcitationConfigError(f"{context} must be a JSON number")
    number = float(value)
    if not math.isfinite(number):
        raise ExcitationConfigError(f"{context} must be finite")
    return number


def _integer(value: object, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ExcitationConfigError(f"{context} must be a JSON integer")
    return value


def _string(value: object, context: str) -> str:
    if not isinstance(value, str):
        raise ExcitationConfigError(f"{context} must be a JSON string")
    return value


def _boolean(value: object, context: str) -> bool:
    if not isinstance(value, bool):
        raise ExcitationConfigError(f"{context} must be a JSON boolean")
    return value
