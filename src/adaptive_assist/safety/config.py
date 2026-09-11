"""Strict standard-library loading for simulation safety limits."""

import json
from pathlib import Path
from typing import cast

from adaptive_assist.safety.supervisor import SafetyLimits

SAFETY_CONFIG_SCHEMA_VERSION = 1


class SafetyConfigError(ValueError):
    """Raised when a simulation safety-limits configuration is invalid."""


def load_safety_limits(path: str | Path) -> SafetyLimits:
    """Load validated deterministic simulation limits from strict JSON."""
    config_path = Path(path)
    try:
        with config_path.open(encoding="utf-8") as config_file:
            raw_config: object = json.load(config_file)
    except json.JSONDecodeError as error:
        raise SafetyConfigError(
            f"Invalid JSON in safety configuration {config_path}: {error.msg}"
        ) from error
    except OSError as error:
        raise SafetyConfigError(
            f"Could not read safety configuration {config_path}: {error}"
        ) from error

    try:
        config = _configuration_object(raw_config)
        _require_exact_fields(config)
        schema_version = _integer(config["schema_version"], "schema_version")
        if schema_version != SAFETY_CONFIG_SCHEMA_VERSION:
            raise ValueError(
                f"schema_version must be {SAFETY_CONFIG_SCHEMA_VERSION}; "
                f"received {schema_version}"
            )
        return SafetyLimits(
            max_abs_assistive_torque_n_m=_number(
                config["max_abs_assistive_torque_n_m"],
                "max_abs_assistive_torque_n_m",
            ),
            min_joint_angle_rad=_number(
                config["min_joint_angle_rad"],
                "min_joint_angle_rad",
            ),
            max_joint_angle_rad=_number(
                config["max_joint_angle_rad"],
                "max_joint_angle_rad",
            ),
            max_abs_joint_velocity_rad_s=_number(
                config["max_abs_joint_velocity_rad_s"],
                "max_abs_joint_velocity_rad_s",
            ),
            fallback_assistive_torque_n_m=_number(
                config["fallback_assistive_torque_n_m"],
                "fallback_assistive_torque_n_m",
            ),
        )
    except SafetyConfigError:
        raise
    except ValueError as error:
        raise SafetyConfigError(
            f"Invalid safety configuration in {config_path}: {error}"
        ) from error


def _configuration_object(value: object) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise SafetyConfigError("safety configuration must be a JSON object")
    return cast(dict[str, object], value)


def _require_exact_fields(config: dict[str, object]) -> None:
    expected_fields = {
        "schema_version",
        "max_abs_assistive_torque_n_m",
        "min_joint_angle_rad",
        "max_joint_angle_rad",
        "max_abs_joint_velocity_rad_s",
        "fallback_assistive_torque_n_m",
    }
    actual_fields = set(config)
    missing_fields = sorted(expected_fields - actual_fields)
    unknown_fields = sorted(actual_fields - expected_fields)
    if missing_fields:
        raise SafetyConfigError(
            f"safety configuration is missing required field(s): "
            f"{', '.join(missing_fields)}"
        )
    if unknown_fields:
        raise SafetyConfigError(
            f"safety configuration contains unsupported field(s): "
            f"{', '.join(unknown_fields)}"
        )


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SafetyConfigError(f"{name} must be a JSON number")
    return float(value)


def _integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise SafetyConfigError(f"{name} must be a JSON integer")
    return value
