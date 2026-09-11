"""Strict standard-library loading for deterministic robustness sweeps."""

import json
from pathlib import Path
from typing import cast

from adaptive_assist.evaluation.robustness import (
    CombinedMismatch,
    MismatchParameter,
    ParameterVariation,
    RobustnessSweepConfig,
)


class RobustnessConfigError(ValueError):
    """Raised when a robustness configuration is malformed or unsupported."""


def load_robustness_sweep_config(path: str | Path) -> RobustnessSweepConfig:
    """Load a strict versioned deterministic model-mismatch configuration."""
    config_path = Path(path)
    try:
        with config_path.open(encoding="utf-8") as config_file:
            raw_config: object = json.load(config_file)
    except json.JSONDecodeError as error:
        raise RobustnessConfigError(
            f"Invalid JSON in robustness configuration {config_path}: {error.msg}"
        ) from error
    except OSError as error:
        raise RobustnessConfigError(
            f"Could not read robustness configuration {config_path}: {error}"
        ) from error

    try:
        return _parse_configuration(raw_config)
    except RobustnessConfigError:
        raise
    except ValueError as error:
        raise RobustnessConfigError(
            f"Invalid robustness configuration in {config_path}: {error}"
        ) from error


def _parse_configuration(raw_config: object) -> RobustnessSweepConfig:
    config = _object(raw_config, "robustness configuration")
    _require_exact_keys(
        config,
        {
            "schema_version",
            "scenario_config_path",
            "impedance_controller_config_path",
            "computed_torque_controller_config_path",
            "safety_limits_config_path",
            "safety_supervision_enabled",
            "varied_parameters",
            "scale_factors",
            "combined_mismatch",
        },
        "robustness configuration",
    )
    combined_data = _object(config["combined_mismatch"], "combined_mismatch")
    _require_exact_keys(
        combined_data,
        {"case_name", "parameter_scale_factors"},
        "combined_mismatch",
    )
    combined_scales = _object(
        combined_data["parameter_scale_factors"],
        "combined_mismatch.parameter_scale_factors",
    )
    supported_parameter_names = {parameter.value for parameter in MismatchParameter}
    _require_only_supported_keys(
        combined_scales,
        supported_parameter_names,
        "combined_mismatch.parameter_scale_factors",
    )

    varied_parameters_data = _array(config["varied_parameters"], "varied_parameters")
    scale_factors_data = _array(config["scale_factors"], "scale_factors")
    varied_parameters = tuple(
        _mismatch_parameter(value, f"varied_parameters[{index}]")
        for index, value in enumerate(varied_parameters_data)
    )
    scale_factors = tuple(
        _positive_number(value, f"scale_factors[{index}]")
        for index, value in enumerate(scale_factors_data)
    )
    combined_variations = tuple(
        ParameterVariation(
            parameter,
            _positive_number(
                combined_scales[parameter.value],
                (f"combined_mismatch.parameter_scale_factors.{parameter.value}"),
            ),
        )
        for parameter in MismatchParameter
        if parameter.value in combined_scales
    )

    return RobustnessSweepConfig(
        schema_version=_integer(config["schema_version"], "schema_version"),
        scenario_config_path=_string(
            config["scenario_config_path"],
            "scenario_config_path",
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
            config["safety_limits_config_path"],
            "safety_limits_config_path",
        ),
        safety_supervision_enabled=_boolean(
            config["safety_supervision_enabled"],
            "safety_supervision_enabled",
        ),
        varied_parameters=varied_parameters,
        scale_factors=scale_factors,
        combined_mismatch=CombinedMismatch(
            case_name=_string(
                combined_data["case_name"],
                "combined_mismatch.case_name",
            ),
            variations=combined_variations,
        ),
    )


def _object(value: object, context: str) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise RobustnessConfigError(f"{context} must be a JSON object")
    return cast(dict[str, object], value)


def _array(value: object, context: str) -> list[object]:
    if not isinstance(value, list):
        raise RobustnessConfigError(f"{context} must be a JSON array")
    return cast(list[object], value)


def _require_exact_keys(
    value: dict[str, object],
    expected_keys: set[str],
    context: str,
) -> None:
    actual_keys = set(value)
    missing_keys = sorted(expected_keys - actual_keys)
    extra_keys = sorted(actual_keys - expected_keys)
    if missing_keys:
        raise RobustnessConfigError(
            f"{context} is missing required field(s): {', '.join(missing_keys)}"
        )
    if extra_keys:
        raise RobustnessConfigError(
            f"{context} contains unsupported field(s): {', '.join(extra_keys)}"
        )


def _require_only_supported_keys(
    value: dict[str, object],
    supported_keys: set[str],
    context: str,
) -> None:
    unknown_keys = sorted(set(value) - supported_keys)
    if unknown_keys:
        raise RobustnessConfigError(
            f"{context} contains unsupported field(s): {', '.join(unknown_keys)}"
        )


def _mismatch_parameter(value: object, context: str) -> MismatchParameter:
    parameter_name = _string(value, context)
    try:
        return MismatchParameter(parameter_name)
    except ValueError as error:
        supported = ", ".join(parameter.value for parameter in MismatchParameter)
        raise RobustnessConfigError(
            f"{context} must be one of: {supported}; received {parameter_name!r}"
        ) from error


def _positive_number(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RobustnessConfigError(f"{context} must be a JSON number")
    number = float(value)
    if number <= 0.0:
        raise RobustnessConfigError(f"{context} must be greater than zero")
    return number


def _integer(value: object, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise RobustnessConfigError(f"{context} must be a JSON integer")
    return value


def _string(value: object, context: str) -> str:
    if not isinstance(value, str):
        raise RobustnessConfigError(f"{context} must be a JSON string")
    return value


def _boolean(value: object, context: str) -> bool:
    if not isinstance(value, bool):
        raise RobustnessConfigError(f"{context} must be a JSON boolean")
    return value
