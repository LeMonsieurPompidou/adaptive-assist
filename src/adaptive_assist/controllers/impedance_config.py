"""Strict standard-library loading for impedance-controller parameters."""

import json
from pathlib import Path
from typing import cast

from adaptive_assist.controllers.impedance import ImpedanceControllerParameters

IMPEDANCE_CONFIG_SCHEMA_VERSION = 1


class ImpedanceConfigError(ValueError):
    """Raised when an impedance-controller configuration is invalid."""


def load_impedance_controller_parameters(
    path: str | Path,
) -> ImpedanceControllerParameters:
    """Load validated impedance gains from a strict JSON configuration."""
    config_path = Path(path)
    try:
        with config_path.open(encoding="utf-8") as config_file:
            raw_config: object = json.load(config_file)
    except json.JSONDecodeError as error:
        raise ImpedanceConfigError(
            f"Invalid JSON in impedance configuration {config_path}: {error.msg}"
        ) from error
    except OSError as error:
        raise ImpedanceConfigError(
            f"Could not read impedance configuration {config_path}: {error}"
        ) from error

    try:
        config = _configuration_object(raw_config)
        _require_exact_fields(config)
        schema_version = _integer(config["schema_version"], "schema_version")
        controller_type = _string(config["controller_type"], "controller_type")
        if schema_version != IMPEDANCE_CONFIG_SCHEMA_VERSION:
            raise ValueError(
                f"schema_version must be {IMPEDANCE_CONFIG_SCHEMA_VERSION}; "
                f"received {schema_version}"
            )
        if controller_type != "impedance":
            raise ValueError(
                f"controller_type must be 'impedance'; received {controller_type!r}"
            )
        return ImpedanceControllerParameters(
            proportional_gain_n_m_per_rad=_number(
                config["proportional_gain_n_m_per_rad"],
                "proportional_gain_n_m_per_rad",
            ),
            derivative_gain_n_m_s_per_rad=_number(
                config["derivative_gain_n_m_s_per_rad"],
                "derivative_gain_n_m_s_per_rad",
            ),
        )
    except ImpedanceConfigError:
        raise
    except ValueError as error:
        raise ImpedanceConfigError(
            f"Invalid impedance configuration in {config_path}: {error}"
        ) from error


def _configuration_object(value: object) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ImpedanceConfigError("impedance configuration must be a JSON object")
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
        raise ImpedanceConfigError(
            f"impedance configuration is missing required field(s): "
            f"{', '.join(missing_fields)}"
        )
    if unknown_fields:
        raise ImpedanceConfigError(
            f"impedance configuration contains unsupported field(s): "
            f"{', '.join(unknown_fields)}"
        )


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ImpedanceConfigError(f"{name} must be a JSON number")
    return float(value)


def _integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ImpedanceConfigError(f"{name} must be a JSON integer")
    return value


def _string(value: object, name: str) -> str:
    if not isinstance(value, str):
        raise ImpedanceConfigError(f"{name} must be a JSON string")
    return value
