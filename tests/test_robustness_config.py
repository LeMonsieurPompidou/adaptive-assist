"""Tests for strict deterministic robustness configuration loading."""

import json
from pathlib import Path
from typing import cast

import pytest

from adaptive_assist.evaluation import (
    ROBUSTNESS_CONFIG_SCHEMA_VERSION,
    MismatchParameter,
    RobustnessConfigError,
    load_robustness_sweep_config,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPOSITORY_ROOT / "configs/robustness/model_mismatch_sweep.json"


def _payload() -> dict[str, object]:
    raw_payload: object = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    return cast(dict[str, object], raw_payload)


def _write_payload(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_nominal_robustness_configuration_loads() -> None:
    configuration = load_robustness_sweep_config(CONFIG_PATH)

    assert configuration.schema_version == ROBUSTNESS_CONFIG_SCHEMA_VERSION
    assert configuration.safety_supervision_enabled
    assert configuration.scale_factors == (0.8, 1.0, 1.2)
    assert configuration.varied_parameters == tuple(MismatchParameter)
    assert configuration.combined_mismatch.case_name == ("combined_moderate_mismatch")
    assert len(configuration.combined_mismatch.variations) == 5


def test_malformed_robustness_configuration_is_rejected(tmp_path: Path) -> None:
    config_path = tmp_path / "malformed.json"
    config_path.write_text("{not valid JSON", encoding="utf-8")

    with pytest.raises(RobustnessConfigError, match="Invalid JSON"):
        load_robustness_sweep_config(config_path)


def test_missing_top_level_field_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    del payload["scale_factors"]
    config_path = tmp_path / "missing.json"
    _write_payload(config_path, payload)

    with pytest.raises(RobustnessConfigError, match="missing required field"):
        load_robustness_sweep_config(config_path)


def test_unknown_top_level_field_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    payload["random_seed"] = 42
    config_path = tmp_path / "unknown.json"
    _write_payload(config_path, payload)

    with pytest.raises(RobustnessConfigError, match="unsupported field"):
        load_robustness_sweep_config(config_path)


def test_unknown_mismatch_parameter_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    payload["varied_parameters"] = ["friction"]
    config_path = tmp_path / "unknown-parameter.json"
    _write_payload(config_path, payload)

    with pytest.raises(RobustnessConfigError, match="must be one of"):
        load_robustness_sweep_config(config_path)


@pytest.mark.parametrize(
    "scale_factors",
    [
        [0.8, 1.2],
        [0.8, 1.0, 1.0],
        [0.0, 1.0, 1.2],
    ],
)
def test_invalid_scale_factor_sets_are_rejected(
    tmp_path: Path,
    scale_factors: list[float],
) -> None:
    payload = _payload()
    payload["scale_factors"] = scale_factors
    config_path = tmp_path / "invalid-scales.json"
    _write_payload(config_path, payload)

    with pytest.raises(RobustnessConfigError, match="scale_factors"):
        load_robustness_sweep_config(config_path)


def test_unknown_combined_parameter_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    combined = cast(dict[str, object], payload["combined_mismatch"])
    combined_scales = cast(dict[str, object], combined["parameter_scale_factors"])
    combined_scales["friction"] = 1.1
    config_path = tmp_path / "unknown-combined.json"
    _write_payload(config_path, payload)

    with pytest.raises(RobustnessConfigError, match="unsupported field"):
        load_robustness_sweep_config(config_path)


def test_combined_case_requires_multiple_parameters(tmp_path: Path) -> None:
    payload = _payload()
    combined = cast(dict[str, object], payload["combined_mismatch"])
    combined["parameter_scale_factors"] = {"inertia_kg_m2": 1.2}
    config_path = tmp_path / "single-combined.json"
    _write_payload(config_path, payload)

    with pytest.raises(RobustnessConfigError, match="at least two"):
        load_robustness_sweep_config(config_path)
