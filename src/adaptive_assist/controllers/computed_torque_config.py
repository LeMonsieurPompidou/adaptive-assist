"""Strict standard-library loading for computed-torque feedback gains."""

import json
from pathlib import Path
from typing import cast

from adaptive_assist.controllers.computed_torque import (
    ComputedTorqueControllerParameters,
)

COMPUTED_TORQUE_CONFIG_SCHEMA_VERSION = 1


class ComputedTorqueConfigError(ValueError):
    """Raised when a computed-torque configuration is invalid."""


def load_computed_torque_controller_parameters(
    path: str | Path,
) -> ComputedTorqueControllerParameters:
    """Load validated computed-torque gains from strict JSON."""
    config_path = Path(path)
    try:
        with config_path.open(encoding="utf-8") as config_file:
            raw_config: object = json.load(config_file)
    except json.JSONDecodeError as error:
        raise ComputedTorqueConfigError(
            f"Invalid JSON in computed-torque configuration {config_path}: {error.msg}"
        ) from error
    except OSError as error:
        raise ComputedTorqueConfigError(
            f"Could not read computed-torque configuration {config_path}: {error}"
        ) from error

    try:
        config = _configuration_object(raw_config)
        _require_exact_fields(config)
        schema_version = _integer(config["schema_version"], "schema_version")
        controller_type = _string(config["controller_type"], "controller_type")
        if schema_version != COMPUTED_TORQUE_CONFIG_SCHEMA_VERSION:
            raise ValueError(
                f"schema_version must be {COMPUTED_TORQUE_CONFIG_SCHEMA_VERSION}; "
                f"received {schema_version}"
            )
        if controller_type != "computed_torque":
            raise ValueError(
                "controller_type must be 'computed_torque'; "
                f"received {controller_type!r}"
            )
        return ComputedTorqueControllerParameters(
            proportional_gain_n_m_per_rad=_number(
                config["proportional_gain_n_m_per_rad"],
                "proportional_gain_n_m_per_rad",
            ),
            derivative_gain_n_m_s_per_rad=_number(
                config["derivative_gain_n_m_s_per_rad"],
                "derivative_gain_n_m_s_per_rad",
            ),
        )
    except ComputedTorqueConfigError:
        raise
    except ValueError as error:
        raise ComputedTorqueConfigError(
            f"Invalid computed-torque configuration in {config_path}: {error}"
        ) from error


def _configuration_object(value: object) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ComputedTorqueConfigError(
            "computed-torque configuration must be a JSON object"
        )
    return cast(dict[str, object], value)


def _require_exact_fields(config: dict[str, object]) -> None:
    expected_fields = {
        "schema_version",
        "controller_type",
        "proportional_gain_n_m_per_rad",
        "derivative_gain_n_m_s_per_rad",
    }
    actual_fields = set(config)
    missing_fields = sorted(expected_fields - actual_fields)
    unknown_fields = sorted(actual_fields - expected_fields)
    if missing_fields:
        raise ComputedTorqueConfigError(
            f"computed-torque configuration is missing required field(s): "
            f"{', '.join(missing_fields)}"
        )
    if unknown_fields:
        raise ComputedTorqueConfigError(
            f"computed-torque configuration contains unsupported field(s): "
            f"{', '.join(unknown_fields)}"
        )


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ComputedTorqueConfigError(f"{name} must be a JSON number")
    return float(value)


def _integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ComputedTorqueConfigError(f"{name} must be a JSON integer")
    return value


def _string(value: object, name: str) -> str:
    if not isinstance(value, str):
        raise ComputedTorqueConfigError(f"{name} must be a JSON string")
    return value
